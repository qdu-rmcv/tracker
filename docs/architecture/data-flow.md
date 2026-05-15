# 数据流

## 图像到控制量

1. `camera_node` 从 SDK 取帧，转换为 `rgb8`，附上 `camera_optical_frame`，通过 `image_transport::CameraPublisher` 同步发布图像和内参。
2. `armor_detector` 等待 `/camera_info` 初始化 PnP，然后对 `/image_raw` 执行检测。
3. 传统检测流程输出灯条、装甲板、数字分类结果；YOLO 流程直接输出装甲板框和关键点。
4. `armor_detector` 对每块装甲板做 PnP，发布相机坐标系下的 `/detector/armors`。
5. `armor_tracker` 通过 TF 将观测变换到 `target_frame`，默认 `odom`，然后进入跟踪状态机。
6. `planning_trajectory` 接收目标状态，读取 `gimbal_odom -> yaw_link` 和 `gimbal_odom -> pitch_link`，解算 pitch/yaw。
7. `rm_serial_driver` 接收 `/trajectory/send`，通过 LibXR Topic 将目标角、开火标志和目标编号发给下位机。

## 下位机到 TF

1. 下位机通过 LibXR 话题 `ahrs_quaternion` 发送云台姿态四元数。
2. `rm_serial_driver` 将四元数转成 pitch/yaw，并发布 `/joint_states`。
3. `robot_state_publisher` 根据 `rm_gimbal.urdf.xacro` 和 `/joint_states` 发布 `/tf`。
4. 识别、跟踪和弹道节点都依赖这棵 TF 树获取相机与云台姿态。

## 英雄吊射相机切换

当 `robot_type == "hero"` 时，部分节点启用吊射逻辑：

- `rm_serial_driver` 从 LibXR `lob_shot` 话题检测上升沿，发布 `/lob_shot_switch`
- `hik_camera` 或 `mindvision_camera` 订阅 `/lob_shot_switch`
- 相机节点在普通相机与 `*_lob` 参数之间切换，并发布 `/camera_switch_done`
- `planning_trajectory` 可加载普通表和吊射表

## 调试数据流

| 调试输出 | 来源 | 用途 |
| -------- | ---- | ---- |
| `/detector/binary_img` | `armor_detector` | 查看二值化效果 |
| `/detector/number_img` | `armor_detector` | 查看数字 ROI |
| `/detector/result_img` | `armor_detector` | 查看最终识别结果 |
| `/detector/debug_lights` | `armor_detector` | 灯条调试数据 |
| `/detector/debug_armors` | `armor_detector` | 装甲板调试数据 |
| `/tracker/info` | `armor_tracker` | 观测与预测偏差 |
| `/trajectory/info` | `planning_trajectory` | 瞄准点和补偿角 |
| `/detector/marker`, `/tracker/marker` | `armor_marker` | RViz/Foxglove 可视化 |
