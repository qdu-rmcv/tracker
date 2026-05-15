# 弹道配置

## 基础参数

```yaml
/planning_trajectory:
  ros__parameters:
    k: 0.092
    bias_time: 0.01
    s_bias: 0.0
    z_bias: 0.0
    pitch_bias: 0.0
    send_frequency: 200.0
    calculate_mode: true
```

| 参数 | 说明 |
| ---- | ---- |
| `k` | 弹道模型参数 |
| `bias_time` | 预测时间补偿 |
| `s_bias` | 水平距离偏置 |
| `z_bias` | 高度偏置 |
| `pitch_bias` | pitch 偏置 |
| `send_frequency` | 输出频率 |
| `calculate_mode` | 是否使用查表 |

## 查表参数

```yaml
table:
  filename: table.bin
  max_x: 13.0
  min_x: 0.0
  max_y: 2.0
  min_y: -1.0
  resolution: 0.01
  filename_lob: table_lob.bin
  max_x_lob: 22.0
  min_x_lob: 0.0
  max_y_lob: 3.0
  min_y_lob: -1.0
  resolution_lob: 0.01
```

弹道表路径会拼到 `rm_vision_bringup/config/` 下。

## 实时线程参数

```yaml
rt:
  use_rt_thread: true
  cpu: 7
  priority: 80
  enable_cpu_affinity: true
  enable_realtime: true
  lock_memory: true
  statistics_interval: 0
```

如果系统不允许实时调度，节点会降级继续运行。

## 输出角 EKF 参数

```yaml
ekf:
  q_yaw: 0.0
  q_pitch: 0.0
  q_jerk: 0.0
  r_yaw: 0.0
  r_pitch: 0.0
```

