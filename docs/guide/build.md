# 编译构建

## 标准编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 常用分包编译

只编译主链路到 bringup：

```bash
colcon build --symlink-install --packages-up-to rm_vision_bringup
```

只编译识别包：

```bash
colcon build --symlink-install --packages-select armor_detector
```

只编译串口通信：

```bash
colcon build --symlink-install --packages-select rm_serial_driver
```

## 识别后端开关

`armor_detector` 默认启用 OpenVINO、关闭 TensorRT：

```bash
colcon build --symlink-install --packages-select armor_detector \
  --cmake-args \
  -DARMOR_DETECTOR_ENABLE_OPENVINO=ON \
  -DARMOR_DETECTOR_ENABLE_TENSORRT=OFF
```

在 Orin 等平台启用 TensorRT：

```bash
colcon build --symlink-install --packages-select armor_detector \
  --cmake-args \
  -DARMOR_DETECTOR_ENABLE_OPENVINO=OFF \
  -DARMOR_DETECTOR_ENABLE_TENSORRT=ON
```

## 测试

运行全部测试：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

运行指定包测试：

```bash
colcon test --packages-select armor_detector --event-handlers console_direct+
```

## 编译注意事项

- `hik_camera` 会根据主机架构链接 `hikSDK/lib/amd64` 或 `hikSDK/lib/arm64`。
- `mindvision_camera` 会根据架构选择 `mindvisionSDK/lib/x64`、`arm64` 等目录，也可以通过 `-DMVSDK_ARCH=<arch>` 覆盖。
- `planning_trajectory` 默认使用 C++20，并包含独立实时线程逻辑。
- `libxr` 作为 ROS2 wrapper 构建时固定为 Linux system/driver，并导出 `libxr::xr` 给串口和标定包使用。
