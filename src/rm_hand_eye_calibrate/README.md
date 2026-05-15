# rm_hand_eye_calibrate 手眼标定包

标定 `rm_gimbal.urdf.xacro` 中 `camera_joint` 的 `xyz` 和 `rpy`，即相机相对于云台 pitch 轴的安装外参。

## 启动

```bash
ros2 launch rm_hand_eye_calibrate hand_eye_calibrate.launch.py
```

用 rqt 或命令行查看调试图像，确认标定板能被检测到、云台欧拉角显示正常：

```bash
ros2 run rqt_image_view rqt_image_view /rm_hand_eye/debug_image
```

## 标定流程

1. 将棋盘格标定板**固定**在相机正前方 1–3m 处
2. 开机后让云台**静止 10 秒以上**，等待 IMU 姿态收敛
3. 转动云台到不同 yaw/pitch 组合位姿，每个位姿**停稳 1–2 秒**后采集：

```bash
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
```

4. 采集 10–20 帧，确保 yaw 覆盖 ±15° 以上，pitch 覆盖 ±10° 以上，避免纯单轴运动
5. 求解：

```bash
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

1. 终端输出可直接粘贴到 xacro 的值。

将 `xyz` 和 `rpy` 复制到 `src/rm_vision/rm_vision_bringup/config/launch_params.yaml` 即可。

如果结果不理想，可以清空样本重新采集：

```bash
ros2 service call /rm_hand_eye/reset std_srvs/srv/Trigger "{}"
```

以上服务也可在 rqt 的 Service Caller 插件中通过 GUI 触发。

## 参数说明

### 串口

| 参数               | 默认值   | 说明                             |
| ------------------ | -------- | -------------------------------- |
| `vid`              | `"16d0"` | USB 设备 Vendor ID               |
| `pid`              | `"1492"` | USB 设备 Product ID              |
| `timestamp_offset` | `0.0`    | 下位机与上位机的时间戳偏移（秒） |

### 话题

| 参数                | 默认值           | 说明         |
| ------------------- | ---------------- | ------------ |
| `image_topic`       | `"/image_raw"`   | 图像话题     |
| `camera_info_topic` | `"/camera_info"` | 相机内参话题 |

### 标定板

| 参数          | 默认值 | 说明             |
| ------------- | ------ | ---------------- |
| `board_cols`  | `11`   | 棋盘格内角点列数 |
| `board_rows`  | `8`    | 棋盘格内角点行数 |
| `square_size` | `0.02` | 方格边长（米）   |

### IMU 安装

| 参数               | 默认值                  | 说明                                                                                    |
| ------------------ | ----------------------- | --------------------------------------------------------------------------------------- |
| `R_gimbal2imubody` | `[1,0,0, 0,1,0, 0,0,1]` | 云台系→IMU body 系的旋转矩阵（3×3 行主序，9 个元素）。若 IMU 坐标轴与云台对齐则为单位阵 |

### 求解算法

| 参数                      | 默认值   | 说明                                                                                        |
| ------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| `handeye_method`          | `"TSAI"` | 手眼标定方法：`TSAI` / `PARK` / `HORAUD` / `ANDREFF` / `DANIILIDIS`                         |
| `use_robot_world_handeye` | `true`   | `true` = `calibrateRobotWorldHandEye`（AX=ZB，推荐），`false` = `calibrateHandEye`（AX=XB） |

### 采集质量检测

| 参数                         | 默认值  | 说明                                                 |
| ---------------------------- | ------- | ---------------------------------------------------- |
| `check_min_blur_score`       | `60.0`  | 图像清晰度下限（Laplacian 方差），低于此值拒绝采集   |
| `check_min_angle_dist`       | `0.087` | 两次采集的最小姿态角距离（rad，≈5°），防止重复位姿   |
| `check_max_quat_angular_vel` | `0.02`  | 四元数帧间角度差阈值（rad，≈1.1°），低于此值视为静止 |
| `max_age_sec`                | `0.5`   | 四元数与图像时间戳的最大偏差（秒），超过则丢弃       |

### 调试图像

| 参数                  | 默认值                       | 说明             |
| --------------------- | ---------------------------- | ---------------- |
| `publish_debug_image` | `true`                       | 是否发布调试图像 |
| `debug_image_topic`   | `"/rm_hand_eye/debug_image"` | 调试图像话题     |

## 调试图像信息

调试图像上会叠加显示：

- **yaw / pitch / roll**：当前云台欧拉角（度），用于验证 `R_gimbal2imubody` 是否正确
- **STATIC / MOVING**：云台运动状态
- **Blur**：图像清晰度分数
- **PnP RMSE**：标定板检测的重投影误差
- **Samples**：已采集的样本数
- 标定板角点检测结果的可视化

## 常见问题

**调试图像上欧拉角方向与实际不符**：`R_gimbal2imubody` 配置错误，需根据 IMU 实际安装方向调整。

**采集被拒绝 "云台正在运动"**：等云台完全停稳后再采集，或适当增大 `check_max_quat_angular_vel`。

**采集被拒绝 "与已有样本位姿过近"**：转动云台到更大角度差的位姿。

**求解结果不稳定**：增加样本数，确保 yaw/pitch 覆盖范围足够大，避免纯单轴运动。

