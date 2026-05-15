#pragma once

#include <cmath>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

// ============================================================================
//  数据结构 —— 与 Armor.msg / Armors.msg 对应
// ============================================================================

struct Pose
{
  struct Point
  {
    double x = 0, y = 0, z = 0;
  };
  struct Quaternion
  {
    double x = 0, y = 0, z = 0, w = 1;
  };
  Point position;
  Quaternion orientation;
};

struct Armor
{
  std::string number;
  std::string type;
  float distance_to_image_center = 0.0f;
  Pose pose;  // 相机坐标系下的位姿
};

struct Armors
{
  std::vector<Armor> armors;
};

// ============================================================================
//  单块装甲板的配置
// ============================================================================

struct ArmorConfig
{
  std::string number;
  std::string type;
  double angular_offset;       // 相对车头朝向的 yaw 偏移 (rad), 绕 Y 轴
  double horizontal_distance;  // 到车体中心的水平距离 (m), 在 XZ 平面
  double height_offset;        // Y 偏移 (m), 正值 = 向下
  double pitch = 0.0;          // 俯仰角 (rad), 正值 = 向上仰
  double roll = 0.0;           // 横滚角 (rad)
};

// ============================================================================
//  观测噪声配置
//  每个函数签名: f(distance) -> sigma (标准差)
// ============================================================================

struct NoiseConfig
{
  std::function<double(double)> position_sigma;     // (m)
  std::function<double(double)> orientation_sigma;  // (rad)

  /// sigma = k0 + k1 * distance
  static NoiseConfig LinearNoise(double k0_pos, double k1_pos, double k0_ori,
                                 double k1_ori)
  {
    return {[k0_pos, k1_pos](double d) { return k0_pos + k1_pos * d; },
            [k0_ori, k1_ori](double d) { return k0_ori + k1_ori * d; }};
  }

  /// 固定标准差 (与距离无关)
  static NoiseConfig ConstantNoise(double sigma_pos, double sigma_ori)
  {
    return {[sigma_pos](double) { return sigma_pos; },
            [sigma_ori](double) { return sigma_ori; }};
  }
};

// ============================================================================
//  RoboMaster 小车仿真类 —— 观测点固定在相机原点 (0,0,0)
// ============================================================================
//
//  坐标系 (OpenCV / PnP 右手系):
//    - X 右, Y 下, Z 前
//    - yaw 绕 Y 轴, 正方向 Z→X
//
//  速度方程 (车体坐标系):
//    vx 横移 (右), vy 升降 (下), vz 前进, omega 偏航角速度
//
// ============================================================================

class RoboMaster
{
 public:
  using VelocityFunc = std::function<double(double)>;

  RoboMaster() = default;

  // ---- 速度方程 (车体坐标系) ----
  void setVelocityX(VelocityFunc f) { vx_func_ = std::move(f); }
  void setVelocityY(VelocityFunc f) { vy_func_ = std::move(f); }
  void setVelocityZ(VelocityFunc f) { vz_func_ = std::move(f); }
  void setAngularVelocity(VelocityFunc f) { omega_func_ = std::move(f); }

  // ---- 装甲板配置 ----
  void addArmor(const ArmorConfig& cfg) { armor_configs_.push_back(cfg); }

  void setArmor(size_t index, const ArmorConfig& cfg)
  {
    if (index >= armor_configs_.size())
      throw std::out_of_range("Armor index out of range");
    armor_configs_[index] = cfg;
  }

  /// 默认的 4 块装甲板
  void setupDefaultArmors(double horizontal_dist = 0.20, double height_offset = 0.0,
                          double pitch = M_PI / 12.0,  // 15°
                          double roll = 0.0)
  {
    armor_configs_ = {
        {"3", "small", 0.0, horizontal_dist, height_offset, pitch, roll},
        {"3", "small", M_PI / 2.0, horizontal_dist, height_offset, pitch, roll},
        {"3", "small", M_PI, horizontal_dist, height_offset, pitch, roll},
        {"3", "small", -M_PI / 2.0, horizontal_dist, height_offset, pitch, roll}};
  }

  /// 3 块装甲板 (120° 均布)
  void setupTriArmors(double horizontal_dist = 0.20, double height_offset = 0.0,
                      double pitch = M_PI / 12.0, double roll = 0.0)
  {
    armor_configs_ = {
        {"outpost", "small", 0.0, horizontal_dist, height_offset, pitch, roll},
        {"outpost", "small", 2.0 * M_PI / 3.0, horizontal_dist, height_offset, pitch,
         roll},
        {"outpost", "small", -2.0 * M_PI / 3.0, horizontal_dist, height_offset, pitch,
         roll}};
  }

  // ---- 仿真控制 ----

  /// 半隐式欧拉积分步进
  void update(double dt)
  {
    yaw_ += omega_func_(time_) * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));  // 归一化到 [-π, π]

    x_ += vx_func_(time_) * dt;
    y_ += vy_func_(time_) * dt;
    z_ += vz_func_(time_) * dt;

    time_ += dt;
  }

  void reset() { time_ = x_ = y_ = z_ = yaw_ = 0; }

  void setInitialPose(double x, double y, double z, double yaw)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    yaw_ = yaw;
  }

  // ---- 核心接口: 获取相机系下装甲板 ----
  //
  //  观测点固定为相机原点。默认仅返回正面朝向原点的可见装甲板；
  //  当 visible_only=false 时返回所有配置的装甲板，可用于真值/可视化。
  //  若传入 noise_cfg, 则根据装甲板到原点的距离叠加高斯噪声。
  //
  Armors getArmors(const NoiseConfig* noise_cfg = nullptr,
                   bool visible_only = true) const
  {
    Armors result;
    result.armors.reserve(armor_configs_.size());

    std::normal_distribution<double> norm(0.0, 1.0);

    for (const auto& cfg : armor_configs_)
    {
      Pose pose = computeArmorPose(cfg);
      if (visible_only && !isVisibleFromOrigin(pose)) continue;

      Armor a;
      a.number = cfg.number;
      a.type = cfg.type;
      a.pose = pose;

      if (noise_cfg)
      {
        double dist = std::sqrt(pose.position.x * pose.position.x +
                                pose.position.y * pose.position.y +
                                pose.position.z * pose.position.z);

        // 位置噪声
        if (noise_cfg->position_sigma)
        {
          double s = noise_cfg->position_sigma(dist);
          if (s > 0.0)
          {
            a.pose.position.x += norm(rng_) * s;
            a.pose.position.y += norm(rng_) * s;
            a.pose.position.z += norm(rng_) * s;
          }
        }

        // 姿态噪声: 装甲板本体系下 roll/pitch/yaw 各加小角度扰动
        if (noise_cfg->orientation_sigma)
        {
          double s = noise_cfg->orientation_sigma(dist);
          if (s > 0.0)
          {
            auto q_dr = quatFromAxisAngle(1, 0, 0, norm(rng_) * s);
            auto q_dp = quatFromAxisAngle(0, 1, 0, norm(rng_) * s);
            auto q_dy = quatFromAxisAngle(0, 0, 1, norm(rng_) * s);
            auto dq = quatMul(quatMul(q_dy, q_dp), q_dr);
            a.pose.orientation = quatNormalize(quatMul(a.pose.orientation, dq));
          }
        }
      }

      result.armors.push_back(std::move(a));
    }
    return result;
  }

  /// 便捷接口 (语义与旧版一致，默认只返回可见装甲板)
  Armors getArmorsWithNoise(const NoiseConfig& noise_cfg,
                            bool visible_only = true) const
  {
    return getArmors(&noise_cfg, visible_only);
  }

  void setNoiseSeed(unsigned int seed) { rng_.seed(seed); }

  // ---- Getters ----
  double time() const { return time_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double yaw() const { return yaw_; }

 private:
  // 仿真状态
  double time_ = 0, x_ = 0, y_ = 0, z_ = 0, yaw_ = 0;

  // 速度方程 (默认全零)
  VelocityFunc vx_func_ = [](double) { return 0.0; };
  VelocityFunc vy_func_ = [](double) { return 0.0; };
  VelocityFunc vz_func_ = [](double) { return 0.0; };
  VelocityFunc omega_func_ = [](double) { return 0.0; };

  std::vector<ArmorConfig> armor_configs_;
  mutable std::mt19937 rng_{std::random_device{}()};

  // -----------------------------------------------------------------------
  //  四元数辅助 (Hamilton)
  // -----------------------------------------------------------------------
  static Pose::Quaternion quatMul(const Pose::Quaternion& a, const Pose::Quaternion& b)
  {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
  }

  static Pose::Quaternion quatFromAxisAngle(double ax, double ay, double az, double angle)
  {
    double s = std::sin(angle / 2.0);
    return {ax * s, ay * s, az * s, std::cos(angle / 2.0)};
  }

  static Pose::Quaternion quatNormalize(const Pose::Quaternion& q)
  {
    double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12) return {0, 0, 0, 1};
    return {q.x / n, q.y / n, q.z / n, q.w / n};
  }

  /// 用四元数旋转一个向量
  static Pose::Point rotateVec(const Pose::Quaternion& q, const Pose::Point& p)
  {
    Pose::Quaternion v{p.x, p.y, p.z, 0.0};
    Pose::Quaternion q_inv{-q.x, -q.y, -q.z, q.w};
    auto r = quatMul(quatMul(q, v), q_inv);
    return {r.x, r.y, r.z};
  }

  // -----------------------------------------------------------------------
  //  计算单块装甲板在相机系下的位姿
  //    内旋 YXZ: 绕 Y 旋 armor_yaw → 绕 X 旋 pitch → 绕 Z 旋 roll
  //    装甲板法线初始朝 +Z
  // -----------------------------------------------------------------------
  Pose computeArmorPose(const ArmorConfig& cfg) const
  {
    Pose p;
    double armor_yaw = yaw_ + cfg.angular_offset;


    p.position.x = x_ + cfg.horizontal_distance * std::sin(armor_yaw);
    p.position.y = y_ + cfg.height_offset;
    p.position.z = z_ + cfg.horizontal_distance * std::cos(armor_yaw);

    auto q_y = quatFromAxisAngle(0, 1, 0, armor_yaw);
    auto q_x = quatFromAxisAngle(1, 0, 0, cfg.pitch);
    auto q_z = quatFromAxisAngle(0, 0, 1, cfg.roll);
    // 修正: 右乘 Ry(+π/2), 使 -X 轴(正面法线) 指向径向外侧, 与 PnP 解算约定一致
    auto q_face = quatFromAxisAngle(0, 1, 0, M_PI_2);
    p.orientation = quatNormalize(quatMul(quatMul(quatMul(q_y, q_x), q_z), q_face));

    return p;
  }

  // -----------------------------------------------------------------------
  //  装甲板是否朝向原点
  //  观测点 = (0,0,0) ⇒ to_observer = -position
  // -----------------------------------------------------------------------
  // 可视半角 (rad), 四块板 90° 间隔时取 ~40° 保证大部分时间只看到一块
  static constexpr double kVisibleHalfAngle = 60.0 * M_PI / 180.0;

  static bool isVisibleFromOrigin(const Pose& pose) 
  {
      // 法线方向: -X 轴为径向外侧 (与 q_face=Ry(+π/2) 配合)
      Pose::Point n = rotateVec(pose.orientation, {-1.0, 0.0, 0.0});

      double dist = std::sqrt(pose.position.x * pose.position.x +
                              pose.position.y * pose.position.y +
                              pose.position.z * pose.position.z);
      if (dist < 1e-9) return false;

      // 原点 -> 装甲板方向 = position
      // 正对着时，法线 n 与 position 同向，夹角为 0
      double cos_angle = -(n.x * pose.position.x + n.y * pose.position.y +
                          n.z * pose.position.z) / dist;

      return cos_angle > std::cos(kVisibleHalfAngle);
  }
};
                                                                                                                                                               