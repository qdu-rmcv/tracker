# 串口配置

## 基本配置

```yaml
/rm_serial_driver:
  ros__parameters:
    vid: "16d0"
    pid: "1492"
    timestamp_offset: 0.0
    send_velocity: true
```

| 参数 | 说明 |
| ---- | ---- |
| `vid` | USB Vendor ID |
| `pid` | USB Product ID |
| `timestamp_offset` | `/joint_states` 时间戳补偿，单位秒 |
| `send_velocity` | 是否向下位机发送 yaw 速度和加速度字段 |
| `robot_type` | `hero` 时启用吊射信号 |

## 时间戳方向

串口节点发布 `/joint_states` 时使用：

```text
stamp = now() - timestamp_offset
```

如果云台姿态相对图像偏快或偏慢，需要结合调车现象调整该值。

## LibXR 通信参数

代码中串口固定为：

| 项 | 值 |
| -- | -- |
| baudrate | `115200` |
| parity | `NO_PARITY` |
| data bits | `8` |
| stop bits | `1` |

设备通过 `vid` / `pid` 查找。

