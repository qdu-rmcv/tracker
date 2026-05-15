# 启动方法

## 实车主链路

```bash
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py robot:=<robot_type>
```

这个 launch 会：

- 切换 `src/rm_vision/rm_vision_bringup/config` 到 `robot` 对应分支
- 启动 `robot_state_publisher`
- 按配置选择 `hik_camera` 或 `mindvision_camera`
- 启动 `armor_detector`、`armor_tracker`、`rm_serial_driver`、`armor_marker`
- 单独启动 `planning_trajectory` 容器，便于绑核和实时循环

## 无下位机调试

```bash
source install/setup.bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

适合只看相机、识别和跟踪，不经过串口控制链路。

## 纯仿真

```bash
source install/setup.bash
ros2 launch rm_vision_bringup sim_bringup.launch.py
```

纯仿真不启动相机、识别和串口。`rm_simulator` 直接发布 `/detector/armors`，并发布静态 TF。

## 真云台 + 仿真目标

```bash
source install/setup.bash
ros2 launch rm_vision_bringup sim_hardware.launch.py robot:=<robot_type>
```

这个模式使用真实串口和真实 `/joint_states`，但目标由 `rm_simulator` 产生，适合验证云台闭环和弹道输出。

## 视频回放

```bash
source install/setup.bash
ros2 launch rm_vision_bringup video.launch.py video_path:=/path/to/video.mp4
```

`video_publisher` 会发布 `/image_raw` 和 `/camera_info`，然后由 `armor_detector` 识别。

## 单独启动相机

海康：

```bash
ros2 launch hik_camera hik_camera.launch.py
```

MindVision：

```bash
ros2 launch mindvision_camera mindvision_camera.launch.py
```

## 单独启动串口

```bash
ros2 launch rm_serial_driver ros2_libxr_launch.py
```

如果设备权限不足，先处理串口权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

实际设备名由 LibXR 根据 `vid` / `pid` 查找，不一定固定为 `/dev/ttyACM0`。

## Foxglove 可视化

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
