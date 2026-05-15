# 常见问题

## 没有 `/detector/armors`

检查顺序：

1. `/image_raw` 是否有数据
2. `/camera_info` 是否有数据
3. `armor_detector` 是否等待 PnP 初始化
4. `detect_color` 是否与目标颜色一致
5. `binary_lower_thres` 是否过高或过低
6. 如果使用 YOLO，确认编译时启用了对应后端

## tracker 一直 LOST

可能原因：

- `/detector/armors` frame 与 TF 不连通
- `target_frame` 配错
- PnP 距离异常，触发 `max_armor_distance` 过滤
- `max_match_distance` 太小

## 弹道节点一直发停止命令

检查：

- `/tracker/target.tracking` 是否为 true
- `gimbal_odom -> yaw_link` 和 `gimbal_odom -> pitch_link` 是否可查
- 弹道表是否存在
- `send_frequency` 是否有效

## 相机启动后没有图像

检查：

- SDK 运行库是否能加载
- `device_index` 或 `device_sn` 是否正确
- USB 权限和带宽
- 曝光或取流超时日志

## 串口没有 `/joint_states`

检查：

- `vid` / `pid` 是否正确
- 下位机是否发送 `ahrs_quaternion`
- LibXR Topic 名是否与下位机一致
- USB 权限是否足够

## TF 不完整

检查：

- `robot_state_publisher` 是否启动
- `/joint_states` 中 joint name 是否是 `pitch_joint`、`yaw_joint`
- `launch_params.yaml` 是否给了合法 `xyz` / `rpy`
- 纯仿真时 `rm_simulator.publish_tf` 是否为 true

## `sim_hardware.launch.py` 找不到仿真包

当前源码中的 `sim_hardware.launch.py` 仿真组件 `package` 字段写成了 `armor_simulator`，而仓库实际包名是 `rm_simulator`。如果启动时报找不到 package，需要把该 launch 中的组件包名改为 `rm_simulator`。
