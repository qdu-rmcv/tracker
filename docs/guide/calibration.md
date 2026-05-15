# 标定入口

## 相机内参标定

启动相机：

```bash
ros2 launch hik_camera hik_camera.launch.py
```

启动 ROS2 相机标定工具：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 11x8 \
  --square 0.02 \
  image:=/image_raw \
  camera:=/hik_camera
```

参数说明：

| 参数 | 含义 |
| ---- | ---- |
| `--size 11x8` | 棋盘格内角点数量，格式为 `列x行` |
| `--square 0.02` | 单个棋盘格边长，单位为米 |

标定结果需要写入对应相机包或 bringup 配置中的 `camera_info.yaml`。

## 手眼标定

启动手眼标定：

```bash
ros2 launch rm_hand_eye_calibrate hand_eye_calibrate.launch.py
```

查看调试图像：

```bash
ros2 run rqt_image_view rqt_image_view /rm_hand_eye/debug_image
```

服务入口：

```bash
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
ros2 service call /rm_hand_eye/reset std_srvs/srv/Trigger "{}"
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

求解结果会输出可直接写入 `rm_gimbal.urdf.xacro` 或 launch 配置的 `xyz` / `rpy`。

## 时间戳调试

相机图像和 IMU 姿态的时间戳需要尽量对齐。调车时可以固定目标不动、晃动车头，观察目标位置估计是否随运动明显漂移。

常用调整项：

| 参数 | 位置 | 说明 |
| ---- | ---- | ---- |
| `timestamp_offset` | `rm_serial_driver` | 下位机姿态时间戳补偿 |
| `timestamp_offset` | `rm_hand_eye_calibrate` | 标定时 IMU 与图像对齐补偿 |

