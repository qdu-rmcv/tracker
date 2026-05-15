# planning_trajectory - 弹道解算包

## 概述

`planning_trajectory` 接收 tracker 输出的目标状态，结合当前云台 yaw/pitch，通过弹道模型计算瞄准角、开火标志和目标编号，并以较高频率发布 `/trajectory/send`。

**路径**: `src/rm_auto_aim/planning_trajectory/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | `planning_trajectory` |
| 组件插件 | `rm_auto_aim::PlanningTrajectoryNode` |
| 可执行文件 | `planning_trajectory_node` |

## 订阅话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/tracker/target` | `auto_aim_interfaces/msg/Target` | 跟踪目标状态 |
| `/current_velocity` | `auto_aim_interfaces/msg/Velocity` | 弹速或当前速度输入 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 当前云台 yaw/pitch |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/trajectory/send` | `auto_aim_interfaces/msg/Send` | 下发给串口节点的控制命令 |
| `/trajectory/info` | `auto_aim_interfaces/msg/TrajectoryInfo` | 瞄准点和补偿角调试信息 |

## 实时循环

节点支持两种循环方式：

| 模式 | 参数 | 说明 |
| ---- | ---- | ---- |
| 独立线程 | `rt.use_rt_thread=true` | 使用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` |
| ROS Timer | `rt.use_rt_thread=false` | 使用 `rclcpp::Timer` |

默认发布频率由 `send_frequency` 决定，默认 `200 Hz`。

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `k` | `0.092` | 空气阻力或弹道模型参数 |
| `bias_time` | `0.01` | 预测时间补偿 |
| `s_bias` | `0.0` | 水平距离偏置 |
| `z_bias` | `0.0` | 高度偏置 |
| `pitch_bias` | `0.0` | pitch 角偏置 |
| `send_frequency` | `200.0` | 控制输出频率 |
| `calculate_mode` | `true` | `true` 使用查表，`false` 使用普通计算 |
| `table.filename` | `table.bin` | 普通弹道表 |
| `table.filename_lob` | `""` | 英雄吊射弹道表 |
| `rt.cpu` | `7` | 实时线程绑核 CPU |
| `rt.priority` | `80` | SCHED_FIFO 优先级 |
| `ekf.q_*`, `ekf.r_*` | `0.0` | 输出角平滑 EKF 参数 |

## 输出逻辑

当没有目标或 `tracking=false` 时，节点发布停止命令：

| 字段 | 值 |
| ---- | -- |
| `is_fire` | `false` |
| `pitch` | `0.0` |
| `yaw` | `0.0` |

当目标有效时，节点调用 `TrajectorySolver::AutoSolveTrajectory`，计算：

- 预测瞄准点 `aim_position`
- 补偿后的 `pitch` / `yaw`
- 是否满足开火条件 `is_fire`
- 当前目标编号 `num`

## 注意事项

- 当前代码保留了上传版本行为：`send_msg.vel_yaw` 和 `send_msg.acc_yaw` 被强制置零。
- 若实时线程没有权限设置 `SCHED_FIFO`、`mlockall` 或 CPU affinity，节点会降级继续运行。
- 查表文件从 `rm_vision_bringup/config/` 读取，因此配置仓库必须存在。

