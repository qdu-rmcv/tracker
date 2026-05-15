# 相机与手眼标定流程

## 相机内参

1. 启动相机节点
2. 打开 `camera_calibration`
3. 按标定工具提示移动棋盘格
4. 保存标定结果
5. 写入对应 `camera_info.yaml`

命令：

```bash
ros2 launch hik_camera hik_camera.launch.py
ros2 run camera_calibration cameracalibrator \
  --size 11x8 \
  --square 0.02 \
  image:=/image_raw \
  camera:=/hik_camera
```

## 手眼标定

1. 固定棋盘格标定板，距离相机 1-3m
2. 开机后等待 IMU 姿态稳定
3. 启动手眼标定节点
4. 打开 `/rm_hand_eye/debug_image`
5. 移动云台到不同 yaw/pitch 组合
6. 每个位姿停稳后调用 `/rm_hand_eye/capture`
7. 采集 10-20 帧后调用 `/rm_hand_eye/solve`
8. 把输出的 `xyz` / `rpy` 写入配置

命令：

```bash
ros2 launch rm_hand_eye_calibrate hand_eye_calibrate.launch.py
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

## 样本质量建议

- yaw 覆盖至少正负十几度
- pitch 覆盖至少正负十度
- 避免只绕单轴运动
- 每次采样前停稳 1-2 秒
- 拒绝模糊样本，不要强行降低清晰度阈值

