#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glob.h>

using namespace cv;
using namespace std;

int main()
{
    // 棋盘参数设置
    const Size BOARD_SIZE(11, 8);  // 棋盘内角点数量 (列数, 行数)
    const float SQUARE_SIZE = 20.0f;  // 每个方格的实际尺寸 (毫米)
    
    string config_path = "../Data/Calibration_R_T.yaml";
    
    //
    //加载存储数据的YAML文件
    FileStorage fs;
    if (!fs.open(config_path, FileStorage::READ)) {
        cerr << "Error: Failed to open YAML file: " << config_path << endl;
        return -1;  // 失败时返回
    } else {
        cout << "open YAML yes" << endl;
    }


    // 创建棋盘的世界坐标系坐标点
    vector<Point3f> objectPoints;
    for (int i = 0; i < BOARD_SIZE.height; i++) {
        for (int j = 0; j < BOARD_SIZE.width; j++) {
            objectPoints.push_back(Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
        }
    }
    
    // 存储所有图像的角点坐标和对应的世界坐标
    vector<vector<Point2f>> imagePointsAll;
    vector<vector<Point3f>> objectPointsAll;
    
    // 获取图像文件列表
    vector<String> imageFiles;
    vector<String> havChessBFiles;
    glob("../Data/*.png", imageFiles);
    
    if (imageFiles.empty()) {
        cout << "错误: 在image文件夹中没有找到PNG图片文件!" << endl;
        return -1;
    }
    
    cout << "找到 " << imageFiles.size() << " 张图片" << endl;
    
    Size imageSize;
    int validImages = 0;
    
    // 处理每张图片
    for (size_t i = 0; i < imageFiles.size(); i++) {
        Mat image = imread(imageFiles[i]);
        if (image.empty()) {
            cout << "无法读取图片: " << imageFiles[i] << endl;
            continue;
        }
        
        // 转换为灰度图
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        imshow("ia",gray);
        waitKey(0);
        
        if (imageSize.width == 0) {
            imageSize = gray.size();
        }
        
        // 查找棋盘角点
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, BOARD_SIZE, corners,
                                         CALIB_CB_ADAPTIVE_THRESH | 
                                         CALIB_CB_NORMALIZE_IMAGE |
                                         CALIB_CB_FAST_CHECK);
        
        if (found) {
            // 亚像素精确化角点位置
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            
            // 存储角点
            imagePointsAll.push_back(corners);
            objectPointsAll.push_back(objectPoints);

            //记录图片的名字
            havChessBFiles.push_back(imageFiles[i]);
            validImages++;
            
            // 绘制角点（可选，用于验证）
            drawChessboardCorners(image, BOARD_SIZE, corners, found);
            
            cout << "图片 " << i + 1 << "/" << imageFiles.size() 
                 << " 处理成功: " << imageFiles[i] << endl;
            
            // 显示结果（可选）
            Mat resized;
            resize(image, resized, Size(800, 600));
            imshow("棋盘角点检测", resized);
            waitKey(100);  // 短暂显示
        } else {
            cout << "图片 " << i + 1 << "/" << imageFiles.size() 
                 << " 未找到棋盘: " << imageFiles[i] << endl;
        }
    }
    
    destroyAllWindows();
    
    if (validImages < 3) {
        cout << "错误: 需要至少3张有效的棋盘图片进行标定，当前只有 " 
             << validImages << " 张" << endl;
        return -1;
    }
    
    cout << "\n开始相机标定，使用 " << validImages << " 张有效图片..." << endl;
    
    // 相机标定
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<Mat> rvecs, tvecs;
    
    double rms = calibrateCamera(objectPointsAll, imagePointsAll, imageSize,
                                cameraMatrix, distCoeffs, rvecs, tvecs);
    
    cout << "\n=== 相机标定结果 ===" << endl;
    cout << "RMS重投影误差: " << rms << " 像素" << endl;
    cout << "\n内参矩阵 (Camera Matrix):" << endl;
    cout << cameraMatrix << endl;
    cout << "\n畸变系数 (Distortion Coefficients):" << endl;
    cout << distCoeffs << endl;
    
    // // 保存标定结果到文件
    // FileStorage fs("camera_calibration.yml", FileStorage::WRITE);
    // fs << "camera_matrix" << cameraMatrix;
    // fs << "distortion_coefficients" << distCoeffs;
    // fs << "image_width" << imageSize.width;
    // fs << "image_height" << imageSize.height;
    // fs << "rms_error" << rms;
    // fs << "valid_images" << validImages;
    // fs.release();
    
    // cout << "\n标定结果已保存到 camera_calibration.yml" << endl;
    
    // 计算标定精度评估
    vector<float> perViewErrors;
    double totalError = 0;
    
    for (size_t i = 0; i < objectPointsAll.size(); i++) {
        vector<Point2f> projectedPoints;
        projectPoints(objectPointsAll[i], rvecs[i], tvecs[i], 
                     cameraMatrix, distCoeffs, projectedPoints);
        
        double error = norm(imagePointsAll[i], projectedPoints, NORM_L2);
        perViewErrors.push_back((float)(error / objectPointsAll[i].size()));
        totalError += error * error;
    }
    
    double meanError = sqrt(totalError / (validImages * BOARD_SIZE.width * BOARD_SIZE.height));
    cout << "平均重投影误差: " << meanError << " 像素" << endl;
    
    cout << "\n=== 标定完成 ===" << endl;
    cout << "建议: RMS误差小于1.0像素表示标定质量良好" << endl;

    // 开始手眼标定
    vector<Mat> Rs_world_to_camera,Ts_world_to_camera;
    Ts_world_to_camera = tvecs;
    for (size_t i = 0; i < rvecs.size(); i++) {
        Mat R;
        Rodrigues(rvecs[i], R);
        Rs_world_to_camera.push_back(R);
    }

    vector<Mat> Rs_base_to_hand,Ts_base_to_hand;
    for(size_t i=0;i<havChessBFiles.size();i++)
    {
        if(havChessBFiles[i].length()<5)
        {
            cerr<<"错误: 文件名不合法: " << havChessBFiles[i] << endl;
            return 0;
        }

        string key = havChessBFiles[i].substr(0, havChessBFiles[i].length() - 4);
        double tdata[3]={0.,0.,0.};
        Mat R,T(3,1,CV_64F,tdata);
        fs[key] >> R;
        Rs_base_to_hand.push_back(R);
        Ts_base_to_hand.push_back(T);
    }
    
    Mat R_hand_to_cam_out, T_hand_to_cam_out,
        R_base_to_world_out, T_base_to_world_out;

    calibrateRobotWorldHandEye(Rs_world_to_camera, Ts_world_to_camera,
                               Rs_base_to_hand, Ts_base_to_hand,
                               R_base_to_world_out, T_base_to_world_out,
                               R_hand_to_cam_out, T_hand_to_cam_out,
                               CALIB_ROBOT_WORLD_HAND_EYE_SHAH
                               );

    cout<<"----------------------------------"<<endl;
    cout<<"手眼标定完成："<<endl;
    cout<<"手到眼的旋转矩阵："<<endl<<R_hand_to_cam_out<<endl;
    cout<<"手到眼的平移向量："<<endl<<T_hand_to_cam_out<<endl;

    fs.release(); // 关闭文件
    return 0;
}
