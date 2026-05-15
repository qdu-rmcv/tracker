# Launch 配置

## launch_params.yaml

该文件由 `rm_vision_bringup/launch/common.py` 读取，典型字段：

```yaml
camera: hik
detector_log_level: INFO
tracker_log_level: INFO
serial_log_level: INFO
trajectory_log_level: INFO
vision_cpu_list: "4-6"
trajectory_cpu_list: "7"
trajectory_use_rt_thread: true
trajectory_rt_cpu: 7
trajectory_rt_priority: 80
odom2camera:
  xyz: '"0.0 0.0 0.0"'
  rpy: '"0.0 0.0 0.0"'
  lob_xyz: '"0.0 0.0 0.0"'
  lob_rpy: '"0.0 0.0 0.0"'
```

## camera 字段

| 值 | 含义 |
| -- | ---- |
| `hik` | 使用 `hik_camera` |
| `mv` | 使用 `mindvision_camera` |

## robot 参数

实车 launch 会用：

```bash
robot:=<robot_type>
```

这个值会：

- 传给各节点的 `robot_type`
- 作为配置仓库分支名执行 `git checkout`
- 控制英雄机器人吊射相关逻辑

## node_params.yaml

该文件应包含所有组件节点的 ROS 参数，例如：

```yaml
/camera_node:
  ros__parameters:
    exposure_time: 2000.0

/armor_detector:
  ros__parameters:
    detector_type: traditional

/armor_tracker:
  ros__parameters:
    target_frame: odom

/planning_trajectory:
  ros__parameters:
    send_frequency: 200.0
```

节点名必须与 launch 中的 `name=` 一致。

