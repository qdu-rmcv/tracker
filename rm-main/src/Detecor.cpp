#include "../include/Detector.hpp"
#include "../include/NumClassifier.hpp"
#include <deque>
#include <iterator>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <cmath>
// #ifndef NDEBUG
// //Debug
// #endif
Detector::Detector(Light::Color color,double confidence,std::string model_path): 
                   color(color),
                   confidence(confidence),
                   classifier(model_path){}

std::vector<Armor> Detector::DectectedArmor(cv::Mat& frame) {

    cv::Mat binary_img = preprocessImage(frame); //预处理图像
    std::deque<Light> lights = FindLight(frame, binary_img); //寻找灯条
    std::deque<Armor> possible_armors = FindArmor(lights); //寻找装甲板
    std::vector<Armor> armors = ClassifyArmor(possible_armors);
    return armors;
}

cv::Mat Detector::preprocessImage(cv::Mat& rgb_img) //图像预处理
{
  cv::cvtColor(rgb_img, this->gray_img, cv::COLOR_RGB2GRAY);
  
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 200, 255, cv::THRESH_BINARY);

  return binary_img;
}


std::deque<Light> Detector::FindLight(const cv::Mat & rgb_img, const cv::Mat & binary_img) //寻找灯条
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::deque<Light> lights;
  for (const auto & contour : contours) {

    //过滤掉小轮廓
    if (contour.size() < 4) continue; //最小点数
    if(cv::contourArea(contour)<30) continue; //最小面积

    //拟合矩形
    cv::RotatedRect rect = cv::minAreaRect(contour);//最小外接矩形
    Light light(rect);

    //过滤掉不符合灯条特征的矩形
    double aspect_ratio = light.length/light.width; //长宽比
    if (aspect_ratio < 2) continue; //长宽比阈值

    if (light.length < 10 || light.width > 100) continue; //长度，宽度阈值

    auto AngleIsOK = [&light]() ->bool
    {
        double tilt_angle = std::atan2(std::abs(light.top.x - light.bottom.x), std::abs(light.top.y - light.bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;

        if (tilt_angle > 30) return false;
        return true;
    };
    if (!AngleIsOK()) continue; //角度阈值

    auto getLightColor = [&rect, &rgb_img]() -> Light::Color
    {
        //getRoi
        std::vector<cv::Point2f> src_rect(4);
        rect.points(src_rect.data());// points() 返回的顺序是 左下 -> 左上 -> 右上 -> 右下
        src_rect.resize(3);//只能是三个点
        cv::Size roi_sz = rect.size;
        
        // 目标图像中的三个对应顶点
        // 源矩形的三个点将被映射到这个新的正放矩形的三个角点
        std::vector<cv::Point2f> aim_rect{
            cv::Point2f(0, roi_sz.height - 1),
            cv::Point2f(0, 0),
            cv::Point2f(roi_sz.width - 1, 0),
        };

        // 计算仿射变换矩阵
        cv::Mat M = cv::getAffineTransform(src_rect, aim_rect);
        // 应用仿射变换
        cv::Mat light_roi;
        cv::warpAffine(rgb_img, light_roi, M, roi_sz,cv::INTER_NEAREST);
        // cv::Scalar light_color=cv::sum(light_roi);
        // return (light_color[0] < light_color[2]) ? Light::Red : Light::Blue;

        int redrate = 0, bluerate = 0;
        for(int i=0;i<light_roi.rows;i++)
        {
            for(int j=0;j<light_roi.cols;j++)
            {
                cv::Vec3b color = light_roi.at<cv::Vec3b>(i,j);
                if(color[2] > color[0])
                    redrate++;
                else
                    bluerate++;
            }
        }
        return (redrate > bluerate) ? Light::Red : Light::Blue;
    };
    if(getLightColor() != this->color) continue; //不是目标颜色
 
    lights.emplace_back(rect);//记录识别到的灯条
    }  

  return lights;
}

std::deque<Armor> Detector::FindArmor(const std::deque<Light> & lights)
{
    std::deque<Armor> armors;
    std::deque<int> LightIndex;//记录配对成功的灯条索引
    std::deque<std::array<int, 2>> armorLightIndex;//记录每个装甲板两个等条的索引
    std::vector<bool> HaxLight(lights.size(),false);//灯条是否配对成功的哈希表
    if(lights.empty()) return armors;

    auto matchIsOk = [](const Light& light1, const Light& light2) -> bool
    {
        //长度匹配
        double biglen = std::max(light1.length, light2.length);
        double smalen = std::min(light1.length,light2.length);
        double rate = smalen / biglen;
        if(rate<0.7) return false;

        //灯条平行匹配
        cv::Point2f L1vec = light1.top-light1.bottom,
                    L2vec = light2.top-light2.bottom;
        double cosAngle = L1vec.dot(L2vec) / (cv::norm(L1vec) * cv::norm(L2vec));
        double Angle = std::abs( std::acos(cosAngle)/CV_PI*180 );
        if(Angle>20) return false;

        //高度匹配
        cv::Point2f toward = L1vec + L2vec;
        cv::Point2f L1ToL2vec = light1.center-light2.center;

        double HighDiff = std::abs(toward.dot(L1ToL2vec)/cv::norm(toward));
        if(HighDiff>(smalen*0.4)) return false;

        //距离匹配
        double distance = cv::norm(L1ToL2vec);
        if(distance>(6*biglen)||distance<(smalen*0.5)) return false;

        //匹配成功
        return true;
    };

    //枚举灯条进行配对并记录配对成功的灯条索引
    for(int i=0;i<lights.size()-1;i++)
    {
        for(int j=i+1;j<lights.size();j++)
        {
            if(matchIsOk(lights[i],lights[j]))
            {
                armors.emplace_back(lights[i], lights[j]);
                armorLightIndex.push_back({i,j});
                if(HaxLight[i]==false) { LightIndex.push_back(i); HaxLight[i]=true;}
                if(HaxLight[j]==false) { LightIndex.push_back(j); HaxLight[j]= true;}     
            }      
        }
    }

    //筛选配对不准确的装甲板
    auto InMind = [](const std::vector<cv::Point2f>& rect,const cv::Point2f& p) ->bool
    {
        // 检查输入是否为四边形
        if (rect.size() != 4) return false;

        // 计算向量(a,b)和向量(a,p)的叉积
        auto cross_product = [](const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& p) -> float {
        return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);};

        // 计算点p与四条边的叉积
        float cp1 = cross_product(rect[0], rect[1], p);
        float cp2 = cross_product(rect[1], rect[2], p);
        float cp3 = cross_product(rect[2], rect[3], p);
        float cp4 = cross_product(rect[3], rect[0], p);

        // 检查叉积结果的符号是否一致。
        bool has_positive = (cp1 > 0) || (cp2 > 0) || (cp3 > 0) || (cp4 > 0);
        bool has_negative = (cp1 < 0) || (cp2 < 0) || (cp3 < 0) || (cp4 < 0);

        // 如果没有同时出现正数和负数，则点在内部或边界上
        return !(has_positive && has_negative);//返回真，表明内部或边界上
    };

    std::deque<Armor> result;
    int num = 0;
    for(auto& armor:armors)
    {
        bool ArmorOK = true;
        for(auto& index:LightIndex)
        {
            if( index==armorLightIndex[num][0] || index==armorLightIndex[num][1]) continue;//如果是自己的灯条跳过
            if( InMind(armor.Lightcorners,lights[index].top) ) {ArmorOK = false;break;}
            if( InMind(armor.Lightcorners,lights[index].center) ) {ArmorOK = false;break;}
            if( InMind(armor.Lightcorners,lights[index].bottom) ) {ArmorOK = false;break;}
        }
        if(ArmorOK) result.push_back(armor);
        num++;
    }
    return result;
}



std::vector<cv::Mat> Detector::ROIArmor(const std::deque<Armor> & armors)
{
    std::vector<cv::Mat> armors_pattern;
    if(armors.empty()) return armors_pattern;
    armors_pattern.reserve(armors.size());
    
    for(const auto& armor:armors)
    {
        const cv::Size roi_sz(20, 28); //裁剪后图像大小
        const cv::Size armor_sz(32,28);
        const int extendLen = 8;
        const int contractWid = 6;
        
        std::vector<cv::Point2f> aim_rect{
            cv::Point2f(-contractWid,extendLen),
            cv::Point2f(armor_sz.width - contractWid - 1, extendLen),
            cv::Point2f(armor_sz.width - contractWid - 1, armor_sz.height - extendLen - 1),
            cv::Point2f(-contractWid, roi_sz.height - extendLen - 1)
        };
        
        // 计算透视变换矩阵
        cv::Mat M = cv::getPerspectiveTransform(armor.Lightcorners, aim_rect,cv::INTER_NEAREST);
        
        // 应用透视变换
        cv::Mat armor_roi;
        cv::warpPerspective(this->gray_img, armor_roi, M, roi_sz);
        
        cv::threshold(armor_roi, armor_roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);//cv::THRESH_OTSU自动计算最优阈值
        
        // cv::imwrite("/home/king/desktop/homework/workindentify/images/roi2.png",armor_roi);
        // cv::waitKey(1);
        armor_roi = armor_roi/255.0;//神经网络输入归一化
        armors_pattern.push_back(armor_roi);
    }
    return armors_pattern;
}

std::vector<Armor> Detector::ClassifyArmor(const std::deque<Armor>& armors)
{
    std::vector<Armor> result;
    if(armors.empty()) return result;
    result.reserve(armors.size());
    
    std::vector<cv::Mat> armors_pattern = this->ROIArmor(armors);
    std::vector<NumClassifier::Ans> ans = classifier.Classify(armors_pattern);
    
    for(int i=0;i<armors.size();i++)
    {
        if(ans[i].confidence<this->confidence) continue;
        
        Armor::Type type = static_cast<Armor::Type>(ans[i].id);
        if(type == Armor::Type::negative) continue;
        // || 
        //    type == Armor::Type::base || 
        //    type == Armor::Type::outpost
        
        result.push_back(armors[i]);
        result.back().confidence = ans[i].confidence;
        result.back().type = static_cast<Armor::Type>(ans[i].id);
    }
    return result;
}


void Detector::ArmorShow(cv::Mat & rgb_img, const std::deque<Armor> & armors)
{
    // if(armors.empty()) {std::cout<<"nononnnnnnnnnnnnnnnnnnnn"<<std::endl;cv::imwrite("/home/king/desktop/homework/workindentify/images/frame.png",rgb_img);}
    for(auto& armor:armors)
    {
        std::vector<cv::Point> Lightcorners;
        for(auto c:armor.Lightcorners) {Lightcorners.push_back(c);}
        std::vector<std::vector<cv::Point>> contours{Lightcorners};

        cv::polylines(rgb_img,contours,1,cv::Scalar(0, 255, 0),3,cv::LINE_AA);
    }
}
void Detector::ArmorShow(cv::Mat & rgb_img, const std::vector<Armor> & armors)
{
    // if(armors.empty()) {std::cout<<"nononnnnnnnnnnnnnnnnnnnn"<<std::endl;cv::imwrite("/home/king/desktop/homework/workindentify/images/frame.png",rgb_img);}
    for(auto& armor:armors)
    {
        std::vector<cv::Point> Lightcorners;
        for(auto c:armor.Lightcorners) {Lightcorners.push_back(c);}
        std::vector<std::vector<cv::Point>> contours{Lightcorners};

        cv::polylines(rgb_img,contours,1,cv::Scalar(0, 255, 0),3,cv::LINE_AA);
        // std::cout<<"id:"<<armor.type<<std::endl;
    }
}