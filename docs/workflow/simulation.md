# 纯仿真流程

## 启动

```bash
source install/setup.bash
ros2 launch rm_vision_bringup sim_bringup.launch.py
```

## 预期节点

| 节点 | 作用 |
| ---- | ---- |
| `rm_simulator` | 发布仿真装甲板和静态 TF |
| `armor_tracker` | 跟踪仿真装甲板 |
| `planning_trajectory` | 解算弹道 |
| `armor_marker` | 可视化 |

## 检查话题

```bash
ros2 topic hz /detector/armors
ros2 topic echo /tracker/target --once
ros2 topic echo /trajectory/info --once
```

## 常用调参

在 `node_params.yaml` 中调整：

```yaml
/rm_simulator:
  ros__parameters:
    publish_rate: 100.0
    robot:
      init_z: 3.0
      armor_count: 4
      omega: 1.0
    noise:
      k1_pos: 0.01
      k1_ori: 0.03
```

## 使用建议

- 先用纯仿真确认 tracker 状态机和弹道输出。
- 再用 `sim_hardware.launch.py` 接入真实串口和云台。
- 最后切回实车相机和识别。

