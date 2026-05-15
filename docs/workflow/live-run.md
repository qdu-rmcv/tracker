# 实车运行流程

## 启动前检查

1. 确认 `src/rm_vision/rm_vision_bringup/config` 已存在并包含目标机器人分支
2. 确认相机 SDK 运行库可加载
3. 确认下位机 USB VID/PID 与配置一致
4. 确认 `camera_info.yaml` 与当前相机匹配
5. 确认 `launch_params.yaml` 中相机外参和相机类型正确

## 启动

```bash
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py robot:=<robot_type>
```

## 验证顺序

| 顺序 | 检查 | 命令 |
| ---- | ---- | ---- |
| 1 | 图像是否发布 | `ros2 topic hz /image_raw` |
| 2 | 相机内参是否发布 | `ros2 topic echo /camera_info --once` |
| 3 | 识别是否输出 | `ros2 topic echo /detector/armors --once` |
| 4 | TF 是否完整 | `ros2 run tf2_tools view_frames` |
| 5 | tracker 是否跟踪 | `ros2 topic echo /tracker/target --once` |
| 6 | 弹道是否输出 | `ros2 topic echo /trajectory/send --once` |
| 7 | 串口是否发布关节 | `ros2 topic hz /joint_states` |

## 可视化

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

重点观察：

- `/detector/result_img`
- `/detector/marker`
- `/tracker/marker`
- `/trajectory/info`
- TF 树

