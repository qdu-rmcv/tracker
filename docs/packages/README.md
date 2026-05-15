# 功能包总览

本章按包解释职责、节点、话题、参数和常见使用方式。阅读顺序建议如下：

1. 先看 [auto_aim_interfaces](auto-aim-interfaces.md)，了解消息字段
2. 再看主链路：识别、跟踪、弹道、串口
3. 最后看相机、仿真、标定和 bringup

## 主链路包

| 包 | 角色 | 一句话说明 |
| -- | ---- | ---------- |
| `armor_detector` | 感知入口 | 把图像变成装甲板三维观测 |
| `armor_tracker` | 状态估计 | 把离散观测变成连续目标状态 |
| `planning_trajectory` | 弹道输出 | 把目标状态变成云台目标角和开火指令 |
| `rm_serial_driver` | 控制边界 | 把 ROS2 控制消息发给下位机 |

## 支撑包

| 包 | 角色 | 一句话说明 |
| -- | ---- | ---------- |
| `auto_aim_interfaces` | 接口 | 定义所有主链路自定义消息 |
| `rm_gimbal_description` | 模型 | 维护云台和相机 TF 结构 |
| `rm_vision_bringup` | 编排 | 组合不同运行模式 |
| `libxr` | 通信库 | 给串口和标定包提供 LibXR |

## 工具包

| 包 | 角色 | 一句话说明 |
| -- | ---- | ---------- |
| `hik_camera` | 海康相机 | 发布图像与内参 |
| `mindvision_camera` | MindVision 相机 | 发布图像与内参 |
| `video_publisher` | 视频回放 | 用视频文件模拟相机 |
| `rm_simulator` | 目标仿真 | 直接生成装甲板观测 |
| `armor_marker` | 可视化 | 发布 RViz/Foxglove marker |
| `rm_hand_eye_calibrate` | 标定 | 求相机到云台的安装外参 |
