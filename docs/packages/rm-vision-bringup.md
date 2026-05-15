# rm_vision_bringup - 启动编排包

## 概述

`rm_vision_bringup` 负责把相机、识别、跟踪、弹道、串口、可视化和 TF 节点组合成不同运行模式。

**路径**: `src/rm_vision/rm_vision_bringup/`

## Launch 文件

| 文件 | 用途 |
| ---- | ---- |
| `vision_bringup.launch.py` | 实车主链路 |
| `no_hardware.launch.py` | 无串口硬件调试 |
| `sim_bringup.launch.py` | 纯仿真 |
| `sim_hardware.launch.py` | 真云台 + 仿真目标 |
| `video.launch.py` | 视频回放 + 识别 |
| `common.py` | 组件构造公共函数 |

## 配置文件

运行时通常需要：

| 文件 | 说明 |
| ---- | ---- |
| `config/launch_params.yaml` | 相机类型、TF 外参、日志等级、绑核参数 |
| `config/node_params.yaml` | 相机、识别、跟踪、弹道、串口等节点参数 |
| `config/camera_info.yaml` | 普通相机内参 |
| `config/camera_info_lob.yaml` | 英雄吊射相机内参 |
| `config/table.bin` | 弹道表 |

当前仓库工作树里没有 `config/` 目录时，需要先初始化或拉取该配置仓库。`vision_bringup.launch.py` 和 `sim_hardware.launch.py` 会进入该目录并执行 `git checkout <robot>`。

## 主链路组件

`common.py` 中的组件构造函数：

| 函数 | 返回组件 |
| ---- | -------- |
| `get_camera_component` | `hik_camera` 或 `mindvision_camera` |
| `get_detector_component` | `armor_detector` |
| `get_tracker_component` | `armor_tracker` |
| `get_trajectory_component` | `planning_trajectory` |
| `get_serial_component` | `rm_serial_driver` |
| `get_marker_component` | `armor_marker` |

## 绑核策略

`vision_bringup.launch.py` 使用 `_safe_taskset_prefix`：

- 视觉容器默认可绑定到 `vision_cpu_list`
- 弹道容器默认可绑定到 `trajectory_cpu_list`
- 若机器 CPU 数不足或 `taskset` 不可用，会自动跳过，不影响启动

