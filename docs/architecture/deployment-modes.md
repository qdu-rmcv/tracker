# 运行模式

## 实车模式

入口：

```bash
ros2 launch rm_vision_bringup vision_bringup.launch.py robot:=<robot_type>
```

适用场景：

- 真实相机
- 真实下位机串口
- 正式调车和比赛运行

节点组合：

| 容器 | 节点 |
| ---- | ---- |
| `vision_container` | 相机、识别、跟踪、串口、marker |
| `trajectory_container` | 弹道解算 |
| 独立进程 | `robot_state_publisher` |

## 无硬件模式

入口：

```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

适用场景：

- 没有下位机或串口设备
- 只调相机、识别和跟踪

## 纯仿真模式

入口：

```bash
ros2 launch rm_vision_bringup sim_bringup.launch.py
```

特点：

- 不启动相机
- 不启动识别
- 不启动串口
- `rm_simulator` 直接发布 `/detector/armors`
- `rm_simulator` 发布静态 TF

## 真云台 + 仿真目标

入口：

```bash
ros2 launch rm_vision_bringup sim_hardware.launch.py robot:=<robot_type>
```

特点：

- 目标来自仿真
- 云台姿态来自真实下位机
- 可验证串口、TF、弹道和云台响应

## 视频回放模式

入口：

```bash
ros2 launch rm_vision_bringup video.launch.py video_path:=/path/to/video.mp4
```

特点：

- `video_publisher` 替代相机
- 默认和 `armor_detector` 组成同一组件容器
- 可选择订阅 `/detector/result_img` 保存识别结果视频

