# rm_simulator - 装甲板仿真包

## 概述

`rm_simulator` 生成虚拟 RoboMaster 目标和装甲板观测，用于不依赖相机和识别算法的跟踪、弹道和可视化调试。

**路径**: `src/rm_simulator/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | `rm_simulator`，类内默认 `armor_simulator` |
| 组件插件 | `rm_auto_aim::RmSimulatorNode` |
| 可执行文件 | `rm_simulator_node` |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | 带噪声的可见装甲板，供 tracker 使用 |
| `/ground_truth/armors` | `auto_aim_interfaces/msg/Armors` | 真值装甲板 |
| `/ground_truth/noisy_armors` | `auto_aim_interfaces/msg/Armors` | 带噪声真值装甲板 |
| `/ground_truth/robot_pose` | `geometry_msgs/msg/PoseStamped` | 车体中心真值 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `publish_tf=true` 时发布静态 TF |

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `publish_rate` | `100.0` | 发布频率 |
| `publish_tf` | `true` | 是否发布静态 TF |
| `ground_truth.publish_all_armors` | `true` | 真值话题是否发布不可见装甲板 |
| `robot.init_x/y/z/yaw` | `0,0,3,pi/2` | 初始位姿 |
| `robot.vx/vy/vz/omega` | `0` | 速度参数 |
| `robot.armor_count` | `4` | 装甲板数量，`3` 表示前哨配置 |
| `robot.horizontal_dist` | `0.20` | 装甲板到中心的水平距离 |
| `robot.height_offset` | `0.0` | 高度偏置 |
| `robot.armor_pitch` | `pi/12` | 装甲板 pitch |
| `noise.k0_pos/k1_pos` | `0 / 0.01` | 位置噪声 |
| `noise.k0_ori/k1_ori` | `0 / 0.03` | 姿态噪声 |
| `noise.seed` | `-1` | 噪声随机种子 |
| `camera.x/y/z` | `0.10/0/0.05` | 仿真 TF 中相机平移 |

## 运行模式

- `sim_bringup.launch.py` 中 `publish_tf=true`，仿真器负责静态 TF。
- `sim_hardware.launch.py` 中 `publish_tf=false`，TF 来自真实 `robot_state_publisher` 和串口 `/joint_states`。

