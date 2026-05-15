# rm_hand_eye_calibrate - 手眼标定包

## 概述

`rm_hand_eye_calibrate` 用于标定相机相对云台 pitch 轴的安装外参，输出可写入 xacro 或 launch 配置的 `xyz` 与 `rpy`。

**路径**: `src/rm_hand_eye_calibrate/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | launch 中为 `hand_eye_calibrate_node`，类内默认 `hand_eye_calibrator_node` |
| 组件插件 | `HandEyeCalibrateNode` |
| 可执行文件 | `rm_hand_eye_calibrate_node` |

## 订阅话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/image_raw` | `sensor_msgs/msg/Image` | 标定图像 |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| LibXR `ahrs_quaternion` | 四元数 | 云台姿态 |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/rm_hand_eye/debug_image` | `sensor_msgs/msg/Image` | 标定板、姿态、清晰度、样本数可视化 |

## 服务

| 服务 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/rm_hand_eye/capture` | `std_srvs/srv/Trigger` | 采集当前样本 |
| `/rm_hand_eye/reset` | `std_srvs/srv/Trigger` | 清空样本 |
| `/rm_hand_eye/solve` | `std_srvs/srv/Trigger` | 求解手眼外参 |

## 质量检查

采集样本时会检查：

- 当前是否检测到棋盘格
- 检测数据是否过旧
- 云台是否静止
- 图像清晰度是否达标
- 新样本与已有样本姿态是否太近

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `board_cols` | `11` | 棋盘格内角点列数 |
| `board_rows` | `8` | 棋盘格内角点行数 |
| `square_size` | `0.02` | 方格边长，单位米 |
| `handeye_method` | `TSAI` | OpenCV classic hand-eye 方法 |
| `use_robot_world_handeye` | `true` | 使用 `calibrateRobotWorldHandEye` |
| `check_min_blur_score` | `60.0` | 清晰度下限 |
| `check_min_angle_dist` | `0.087` | 样本间最小角距离 |
| `check_max_quat_angular_vel` | `0.02` | 静止判定阈值 |
| `max_age_sec` | `0.5` | 图像与 IMU 姿态最大时间差 |

## 启动

```bash
ros2 launch rm_hand_eye_calibrate hand_eye_calibrate.launch.py
```

## 服务调用

```bash
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

