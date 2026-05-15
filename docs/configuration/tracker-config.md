# 跟踪配置

## 通用参数

```yaml
/armor_tracker:
  ros__parameters:
    target_frame: odom
    max_armor_distance: 10.0
```

| 参数 | 说明 |
| ---- | ---- |
| `target_frame` | tracker 输出坐标系 |
| `max_armor_distance` | 过滤过远装甲板 |

## 匹配与状态机

```yaml
tracker:
  max_match_distance: 0.15
  max_match_yaw_diff: 1.0
  tracking_thres: 5
  lost_time_thres: 0.3
  change_time_thres: 0.3
  v_yaw_armor_threshold: 0.4
  v_yaw_full_threshold: 0.6
  radius_min: 0.12
  radius_max: 0.4
  default_init_radius: 0.26
```

## EKF 过程噪声

```yaml
ekf:
  s2_q_x_full: 0.1
  s2_q_y_full: 0.1
  s2_q_z_full: 0.1
  s2_q_yaw_full: 2.0
  s2_q_r_full: 80.0
  s2_q_x_armor: 0.1
  s2_q_y_armor: 0.1
  s2_q_z_armor: 0.1
  s2_q_xy_outpost: 0.005
  s2_q_z_outpost: 0.005
  s2_q_yaw_outpost: 2.0
```

## 测量噪声

```yaml
ekf:
  r_ypd_yaw_std: 0.008
  r_ypd_pitch_std: 0.010
  r_ypd_distance_std_scale: 0.010
  r_armor_yaw_std: 0.10
```

## 前哨站参数

```yaml
outpost:
  outpost_r: 0.2765
  outpost_dz: 0.1
  outpost_cast_threshold: 0.18
  outpost_vyaw_abs: 2.513
  outpost_static_threshold: 1.5
  outpost_learning_frames: 100
  outpost_zc_stable_count: 50
  outpost_idx_geo_margin: 0.15
```

