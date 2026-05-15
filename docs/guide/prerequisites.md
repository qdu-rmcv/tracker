# 环境依赖

## 基础环境

| 项目 | 建议版本 | 说明 |
| ---- | -------- | ---- |
| Ubuntu | 22.04 | 项目 README 中的测试环境 |
| ROS2 | Humble | 当前 launch 和 package 依赖面向 Humble |
| 编译器 | GCC 11+ | C++17 / C++20 混合工程 |
| 构建工具 | colcon | ROS2 标准构建工具 |
| Python | 3.10 | ROS2 Humble 默认环境 |

## ROS2 依赖

常用依赖包括：

- `rclcpp`, `rclcpp_components`
- `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `std_srvs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
- `image_transport`, `image_transport_plugins`
- `camera_info_manager`, `camera_calibration`
- `cv_bridge`, `vision_opencv`
- `message_filters`
- `robot_state_publisher`, `xacro`

安装依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 外部库

| 依赖 | 用途 | 备注 |
| ---- | ---- | ---- |
| OpenCV | 图像处理、PnP、手眼标定、视频读写 | 多数视觉包直接依赖 |
| Eigen3 | EKF、坐标计算、LibXR wrapper | `libxr` 和跟踪算法使用 |
| OpenVINO | YOLO 检测后端 | `ARMOR_DETECTOR_ENABLE_OPENVINO=ON` 时需要 |
| CUDA / TensorRT | YOLO TensorRT 后端 | `ARMOR_DETECTOR_ENABLE_TENSORRT=ON` 时需要 |
| 海康 MVS SDK | `hik_camera` | 仓库内带头文件和运行库 |
| MindVision SDK | `mindvision_camera` | 仓库内按架构放置 `libMVSDK.so` |
| libudev / libnm | LibXR Linux 驱动 | `libxr` 的 Linux 后端会查找 |

## 推荐安装项

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-camera-calibration \
  ros-humble-foxglove-bridge \
  ros-humble-image-transport-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

如果 `libxr` 构建时提示缺少 Linux 网络或设备库：

```bash
sudo apt install -y libudev-dev libwpa-client-dev libnm-dev
```

## 仓库准备

首次 clone 后建议启用仓库 hook，并初始化子模块：

```bash
git config core.hooksPath .githooks
git submodule update --init --recursive
```

`rm_vision_bringup` 运行时依赖 `config/` 下的机器人分支配置。当前工作树没有该配置目录时，`vision_bringup.launch.py` 会在启动阶段执行 `git checkout <robot>`，因此需要先确保配置仓库已经拉取到 `src/rm_vision/rm_vision_bringup/config`。
