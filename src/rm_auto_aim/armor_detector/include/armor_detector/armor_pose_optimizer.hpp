#ifndef ARMOR_DETECTOR__ARMOR_POSE_OPTIMIZER_HPP_
#define ARMOR_DETECTOR__ARMOR_POSE_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <array>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

class ArmorPoseOptimizer
{
 public:
  struct Params
  {
    enum class OptimizeMethod : uint8_t
    {
      LM = 0,
      RANGE = 1,
      RANGE_LM = 2,  // 在范围搜索基础上以最优解为起点再迭代
    };
    OptimizeMethod optimize_method = OptimizeMethod::RANGE_LM;

    // 地面兵种装甲板安装 pitch
    double standard_pitch_deg = 15.0;
    // 前哨站装甲板安装 pitch
    double outpost_pitch_deg = -15.0;

    // 先验约束容差（度）
    double max_pitch_deviation = 10.0;
    double max_roll_deviation = 10.0;

    // 优化方法选择

    // LM 优化参数
    int max_iterations = 20;             // 最大迭代次数
    double convergence_eps = 1e-6;       // 收敛阈值：参数更新量的范数
    double cost_convergence_eps = 1e-8;  // 收敛阈值：代价函数相对变化量
    double initial_lambda = 1e-3;        // 初始阻尼因子
    double lambda_scale_up = 10.0;       // 失败时阻尼放大倍数
    double lambda_scale_down = 10.0;     // 成功时阻尼缩小倍数

    // 范围搜索参数
    bool range_fix_t_cam_ = false;  // 范围搜索时是否固定 t_cam 不变，仅优化 yaw
    double range_search_half_range_deg = 70.0;  // 粗搜索半范围（度）
    double range_search_coarse_step_deg = 1.0;  // 粗搜索步长（度）
    double range_search_fine_range_deg =
        2.0;  // 精搜索半范围（度），在粗搜索最优解附近 ±2度
    double range_search_fine_step_deg = 0.1;  // 精搜索步长（度）
  };

  explicit ArmorPoseOptimizer(const Params& params);

  /// @brief 设置相机内参
  /// @param camera_matrix 3x3 相机内参矩阵
  /// @param dist_coeffs 畸变系数向量
  void SetCameraIntrinsics(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

  /// @brief 设置从相机光学坐标系到惯性系的旋转
  /// @param R_gimbal_cam 从相机系到惯性系的旋转矩阵
  void SetCameraToGimbalRotation(const Eigen::Matrix3d& R_gimbal_cam);

  /// @brief 进行优化，算法流程入口
  /// @param armor   检测到的装甲板
  /// @param rvec    [in/out] 旋转向量（相机系）
  /// @param tvec    [in/out] 平移向量（相机系）
  /// @return true 表示优化成功；false 表示先验不满足或优化失败
  bool Optimize(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);

  void SetArmorNumber(bool is_outpost) { is_outpost_ = is_outpost; }

 private:
  /// 将 cv::Mat 旋转向量转为 3x3 旋转矩阵
  static Eigen::Matrix3d RvecToRotationMatrix(const cv::Mat& rvec);

  /// 将 3x3 旋转矩阵转为 cv::Mat 旋转向量
  static cv::Mat RotationMatrixToRvec(const Eigen::Matrix3d& R);

  /// 欧拉角分解
  /// @return [yaw, pitch, roll]（弧度）
  static Eigen::Vector3d DecomposeZYX(const Eigen::Matrix3d& R);

  /// 构造绕 z 轴的旋转矩阵
  static Eigen::Matrix3d Rz(double yaw);

  /// Rz 对 yaw 的解析导数
  static Eigen::Matrix3d DRzDyaw(double yaw);

  /// 绕 y 轴旋转矩阵
  static Eigen::Matrix3d Ry(double pitch);

  /// 根据装甲板类型构造 4 个物体顶点坐标，与pnp保持一致
  static std::array<Eigen::Vector3d, 4> ExtractObjectPoints(ArmorType type);

  /// 构造相机系下的旋转矩阵
  Eigen::Matrix3d BuildCameraRotation(double yaw, double pitch_prior) const;

  /// 将 4 个物体点经刚体变换后做针孔投影，输出像素坐标
  /// @return false 若存在点在相机后方
  bool ProjectPoints(const Eigen::Matrix3d& R_cam, const Eigen::Vector3d& t_cam,
                     const std::array<Eigen::Vector3d, 4>& obj_points,
                     std::array<Eigen::Vector2d, 4>& projected) const;

  /// 计算给定 (yaw, pitch_prior, t_cam) 下的重投影误差平方和
  /// @return 重投影误差平方和；若有点在相机后方则返回 1e9
  double ComputeReprojectionError(double yaw, double pitch_prior,
                                  const Eigen::Vector3d& t_cam,
                                  const std::array<Eigen::Vector3d, 4>& obj_points,
                                  const std::array<Eigen::Vector2d, 4>& img_points_ud);

  /// 检查先验约束是否满足，并提取初始 yaw 和匹配到的 pitch 先验
  /// @param R_cam       相机系下的旋转矩阵
  /// @param yaw_init    [out] 惯性系下的初始 yaw
  /// @param pitch_prior [out] 匹配到的装甲板安装 pitch 先验（弧度）
  /// @return true 表示先验约束满足
  bool CheckConstraint(const Eigen::Matrix3d& R_cam, double& yaw_init,
                       double& pitch_prior);

  /// 去畸变图像点，返回去畸变后的像素坐标
  std::array<Eigen::Vector2d, 4> UndistortPoints(
      const std::array<cv::Point2f, 4>& points);

  /// 计算残差与雅可比
  /// @return false 表示存在点在相机后方
  bool ComputeResidualAndJacobian(double yaw, double pitch_prior,
                                  const Eigen::Vector3d& t_cam,
                                  const std::array<Eigen::Vector3d, 4>& obj_points,
                                  const std::array<Eigen::Vector2d, 4>& img_points_ud,
                                  Eigen::Matrix<double, 8, 1>& residual,
                                  Eigen::Matrix<double, 8, 4>& jacobian);

  /// LM 迭代求解
  bool RunLM(double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
             const std::array<Eigen::Vector3d, 4>& obj_points,
             const std::array<Eigen::Vector2d, 4>& img_points_ud);

  bool RunRangeSolve(double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
                     const std::array<Eigen::Vector3d, 4>& obj_points,
                     const std::array<Eigen::Vector2d, 4>& img_points_ud,
                     double* best_error_out = nullptr);

  bool SearchYaw(double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
                 const std::array<Eigen::Vector3d, 4>& obj_points,
                 const std::array<Eigen::Vector2d, 4>& img_points_ud,
                 double* best_error_out);
  bool SearchYawWithT(double& yaw, double pitch_prior, Eigen::Vector3d& t_cam,
                      const std::array<Eigen::Vector3d, 4>& obj_points,
                      const std::array<Eigen::Vector2d, 4>& img_points_ud,
                      double* best_error_out);

  /// 固定旋转矩阵，解析求解最优平移向量
  /// @param rotated_points 预计算的旋转后的物体点坐标
  /// @return false 若线性系统退化
  bool SolveTranslationForFixedRotation(
      const std::array<Eigen::Vector3d, 4>& rotated_points,
      const std::array<Eigen::Vector2d, 4>& img_points_ud, Eigen::Vector3d& t_out) const;

  /// 对 yaw 解析求解最优平移后计算重投影误差
  /// @return 重投影误差平方和；若退化则返回 1e9
  double EvaluateCandidateYaw(double yaw, double pitch_prior,
                              const std::array<Eigen::Vector3d, 4>& obj_points,
                              const std::array<Eigen::Vector2d, 4>& img_points_ud,
                              Eigen::Vector3d& t_out);

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 230;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  // Unit: m
  static constexpr double SMALL_HALF_Y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  static constexpr double SMALL_HALF_Z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  static constexpr double LARGE_HALF_Y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  static constexpr double LARGE_HALF_Z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  Params params_;

  bool is_outpost_ = false;  // 是否为前哨站装甲板

  // 装甲板安装 pitch 先验
  double ground_pitch_rad_;
  double outpost_pitch_rad_;

  // 相机内参
  cv::Mat cv_camera_matrix_;
  cv::Mat cv_dist_coeffs_;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  bool intrinsics_set_ = false;

  // 坐标系变换
  Eigen::Matrix3d r_gimbal_cam_;  // 相机系 → 惯性系 的旋转
  Eigen::Matrix3d r_cam_gimbal_;  // 惯性系 → 相机系 的旋转
  bool transform_set_ = false;

  // 范围搜索参数
  double half_range_rad_;
  double coarse_step_rad_;
  double fine_range_rad_;
  double fine_step_rad_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_POSE_OPTIMIZER_HPP_