# 话题列表

## 主链路话题

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
| ---- | ---- | ------ | ------ | ---- |
| `/image_raw` | `sensor_msgs/msg/Image` | 相机 / 视频 | `armor_detector`, `rm_hand_eye_calibrate` | 图像 |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机 / 视频 | `armor_detector`, `rm_hand_eye_calibrate` | 相机内参 |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | `armor_detector` / `rm_simulator` | `armor_tracker`, `armor_marker` | 装甲板观测 |
| `/tracker/target` | `auto_aim_interfaces/msg/Target` | `armor_tracker` | `planning_trajectory`, `armor_marker` | 跟踪目标 |
| `/trajectory/send` | `auto_aim_interfaces/msg/Send` | `planning_trajectory` | `rm_serial_driver` | 控制下发 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `rm_serial_driver` | `robot_state_publisher` | 云台关节状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | 识别、跟踪、弹道 | 动态 TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` / `rm_simulator` | 识别、跟踪、弹道 | 静态 TF |

## 调试与可视化话题

| 话题 | 类型 | 发布者 | 说明 |
| ---- | ---- | ------ | ---- |
| `/detector/debug_lights` | `DebugLights` | `armor_detector` | 灯条调试 |
| `/detector/debug_armors` | `DebugArmors` | `armor_detector` | 装甲板调试 |
| `/detector/binary_img` | `sensor_msgs/msg/Image` | `armor_detector` | 二值化图像 |
| `/detector/number_img` | `sensor_msgs/msg/Image` | `armor_detector` | 数字 ROI |
| `/detector/result_img` | `sensor_msgs/msg/Image` | `armor_detector` | 识别结果图 |
| `/tracker/info` | `TrackerInfo` | `armor_tracker` | 跟踪误差 |
| `/trajectory/info` | `TrajectoryInfo` | `planning_trajectory` | 弹道调试 |
| `/detector/marker` | `MarkerArray` | `armor_marker` | 检测 marker |
| `/tracker/marker` | `MarkerArray` | `armor_marker` | 跟踪与瞄准 marker |

## 特殊话题

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
| ---- | ---- | ------ | ------ | ---- |
| `/lob_shot_switch` | `std_msgs/msg/Bool` | `rm_serial_driver` | 相机节点 | 英雄吊射切换触发 |
| `/camera_switch_done` | `std_msgs/msg/Bool` | 相机节点 | 预留 | 相机切换完成 |
| `/current_velocity` | `Velocity` | 外部 / 下位机扩展 | `planning_trajectory` | 弹速输入 |
| `/rm_hand_eye/debug_image` | `sensor_msgs/msg/Image` | `rm_hand_eye_calibrate` | 可视化工具 | 手眼标定调试图 |
| `/ground_truth/armors` | `Armors` | `rm_simulator` | 可视化工具 | 仿真真值 |
| `/ground_truth/noisy_armors` | `Armors` | `rm_simulator` | 可视化工具 | 带噪声真值 |
| `/ground_truth/robot_pose` | `PoseStamped` | `rm_simulator` | 可视化工具 | 车体中心真值 |

