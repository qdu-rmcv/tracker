# 消息定义

## Armor

文件：`auto_aim_interfaces/msg/Armor.msg`

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `number` | `string` | 装甲板数字或类别，如 `1`、`outpost`、`base` |
| `type` | `string` | `small` / `large` / `outpost` 等 |
| `distance_to_image_center` | `float32` | 距离图像中心的像素或归一化距离 |
| `pose` | `geometry_msgs/Pose` | 装甲板位姿 |

## Armors

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `header` | `std_msgs/Header` | 时间戳与 frame |
| `armors` | `Armor[]` | 一帧中识别到的装甲板 |

## Target

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `header` | `std_msgs/Header` | 输出坐标系，通常是 `odom` |
| `tracking` | `bool` | 是否正在有效跟踪 |
| `type` | `string` | 目标装甲板类型 |
| `num` | `int8` | 目标编号 |
| `armors_num` | `int32` | 目标装甲板数量 |
| `outpost_idx` | `int32` | 前哨站当前索引 |
| `position` | `geometry_msgs/Point` | 目标中心或装甲板位置 |
| `velocity` | `geometry_msgs/Vector3` | 线速度 |
| `yaw` | `float64` | 目标 yaw |
| `v_yaw` | `float64` | 目标 yaw 角速度 |
| `radius_1`, `radius_2` | `float64` | 装甲板半径 |
| `dz` | `float64` | 双层装甲板高度差 |
| `is_switchtable` | `bool` | 弹道表切换标志 |
| `state` | `int8` | tracker 状态 |
| `is_center` | `bool` | `position` 是否表示目标中心 |

## Send

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `header` | `std_msgs/Header` | 预留 |
| `is_fire` | `bool` | 是否开火 |
| `pitch` | `float64` | 目标 pitch |
| `yaw` | `float64` | 目标 yaw |
| `vel_yaw` | `float64` | yaw 速度前馈 |
| `acc_yaw` | `float64` | yaw 加速度前馈 |
| `idx` | `int8` | 目标装甲板索引 |
| `num` | `int8` | 目标编号 |

## TrackerInfo

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `position_diff` | `float64` | 观测与预测位置差 |
| `yaw_diff` | `float64` | 观测与预测 yaw 差 |
| `position` | `geometry_msgs/Point` | 未滤波观测位置 |
| `yaw` | `float64` | 未滤波观测 yaw |
| `outpost_idx` | `int32` | 前哨站索引 |

## TrajectoryInfo

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `aim_position` | `geometry_msgs/Point` | 预测瞄准点 |
| `gimbal_yaw` | `float64` | 当前云台 yaw |
| `gimbal_pitch` | `float64` | 当前云台 pitch |
| `idx` | `int64` | 当前命中的装甲板索引 |
| `bc_yaw` | `float64` | 弹道补偿 yaw |
| `bc_pitch` | `float64` | 弹道补偿 pitch |

## 调试消息

| 消息 | 字段 | 说明 |
| ---- | ---- | ---- |
| `DebugLight` | `center_x`, `is_light`, `ratio`, `angle` | 单个灯条调试 |
| `DebugLights` | `DebugLight[] data` | 多个灯条 |
| `DebugArmor` | `center_x`, `type`, `light_ratio`, `center_distance`, `angle` | 单个装甲板调试 |
| `DebugArmors` | `DebugArmor[] data` | 多个装甲板 |
| `Velocity` | `header`, `velocity` | 弹速或速度输入 |

