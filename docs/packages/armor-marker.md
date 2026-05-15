# armor_marker - 可视化包

## 概述

`armor_marker` 把检测、跟踪和弹道信息转换成 `visualization_msgs/msg/MarkerArray`，用于 RViz 或 Foxglove 调试。

**路径**: `src/rm_auto_aim/armor_marker/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | `armor_marker` |
| 组件插件 | `rm_auto_aim::ArmorMarkerNode` |
| 可执行文件 | `armor_marker_node` |

## 订阅话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | 默认检测结果，可由参数改为真值话题 |
| `/tracker/target` | `auto_aim_interfaces/msg/Target` | 跟踪状态 |
| `/trajectory/info` | `auto_aim_interfaces/msg/TrajectoryInfo` | 瞄准点 |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/detector/marker` | `visualization_msgs/msg/MarkerArray` | 检测装甲板和编号 |
| `/tracker/marker` | `visualization_msgs/msg/MarkerArray` | 目标中心、速度、预测装甲板和瞄准点 |

## 参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `detector_marker.armors_topic` | `/detector/armors` | 检测 marker 的输入话题 |

## 使用建议

- 纯仿真时可把 `detector_marker.armors_topic` 改为 `/ground_truth/armors`，查看全部真值装甲板。
- 实车调试时默认看 `/detector/armors`，可以直接确认识别输出是否稳定。

