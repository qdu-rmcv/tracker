#ifndef ARMOR_PROCESSOR__TRACKER_PARAMS_HPP_
#define ARMOR_PROCESSOR__TRACKER_PARAMS_HPP_

#include <cmath>

namespace rm_auto_aim
{

struct TrackerParams
{
  // ---------- 匹配 ----------
  double max_match_distance = 0.15;
  double max_match_yaw_diff = 1.0;
  int    tracking_thres = 5;
  // 由 node 根据 dt 计算后填入
  int    lost_thres = 30;
  int    change_thres = 30;
  // 时间相关原始参数
  double lost_time_thres = 0.3;
  double change_time_thres = 0.3;

  // ---------- 模型选择(迟滞) ----------
  // 当前在装甲板模型下：|v_yaw| > v_yaw_full_threshold 时才切到整车
  // 当前在整车模型下：|v_yaw| < v_yaw_armor_threshold 时才切到装甲板
  double v_yaw_armor_threshold = 0.4;
  double v_yaw_full_threshold = 0.6;

  // ---------- 半径 ----------
  double radius_min = 0.12;
  double radius_max = 0.4;
  double default_init_radius = 0.26;

  // ---------- 整车 EKF 过程噪声 ----------
  double s2_q_x_full = 0.1;
  double s2_q_y_full = 0.1;
  double s2_q_z_full = 0.1;
  double s2_q_yaw_full = 2.0;
  double s2_q_r_full = 80.0;

  // ---------- 装甲板 EKF (CV) 过程噪声 ----------
  double s2_q_x_armor = 0.1;
  double s2_q_y_armor = 0.1;
  double s2_q_z_armor = 0.1;

  // ---------- 前哨站 EKF 过程噪声 ----------
  double s2_q_xy_outpost = 0.005;
  double s2_q_z_outpost = 0.005;
  double s2_q_yaw_outpost = 2.0;

  // ---------- 前哨站常量 / 行为参数 ----------
  double outpost_r = 0.2765;
  double outpost_dz = 0.1;
  double outpost_cast_threshold = 0.18;
  double outpost_vyaw_abs = 0.8 * M_PI;       // 旋转模式下的 |v_yaw| 钳制值
  double outpost_static_threshold = 1.5;      // 学习期后判定 旋转/静止 的 |v_yaw| 阈值
  int    outpost_learning_frames = 100;       // 开局学习期帧数
  int    outpost_zc_stable_count = 50;        // z_c 稳定后启用几何法 idx 判定
  double outpost_idx_geo_margin = 0.15;       // 几何法 idx 模糊区半宽 (0..0.5)

  // ---------- 测量噪声 (ypd) ----------
  double r_ypd_yaw_std = 0.008;
  double r_ypd_pitch_std = 0.010;
  double r_ypd_distance_std_scale = 0.010;
  double r_armor_yaw_std = 0.10;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_PARAMS_HPP_
