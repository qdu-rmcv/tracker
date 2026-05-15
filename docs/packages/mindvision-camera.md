# mindvision_camera - MindVision 相机驱动包

## 概述

`mindvision_camera` 封装 MindVision SDK，功能与 `hik_camera` 类似，发布图像和内参，并支持按设备索引或序列号选择相机。

**路径**: `src/mindvision_camera/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | launch 中为 `camera_node`，类内默认 `mindvision_camera_node` |
| 组件插件 | `MindVisionCamera::MindVisionCameraNode` |
| 可执行文件 | `mindvision_camera_node` |

## 发布与订阅

| 方向 | 话题 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 发布 | `image_raw` | `sensor_msgs/msg/Image` | RGB 图像 |
| 发布 | `camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| 发布 | `/camera_switch_done` | `std_msgs/msg/Bool` | 英雄相机切换完成 |
| 订阅 | `/lob_shot_switch` | `std_msgs/msg/Bool` | 英雄相机切换触发 |

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `camera_name` | `gimbal_camera` | 相机名称 |
| `camera_info_url` | `package://mindvision_camera/config/camera_info.yaml` | 内参路径 |
| `device_index` | `0` | 设备索引 |
| `device_sn` | `""` | 设备序列号，非空时用于精确匹配 |
| `frame_id` | `camera_optical_frame` | 图像 frame |
| `exposure_time` | `1000.0` | 曝光时间 |
| `gain` | `15.0` | 增益 |
| `gamma` | `75` | SDK gamma 原始值 |
| `frame_rate` | `249.0` | 帧率参数 |
| `grab_timeout_ms` | `20` | 取流超时 |
| `rotate` | `0` | 旋转模式 |

英雄机器人额外支持 `device_index_lob`、`device_sn_lob`、`camera_info_url_lob`、`frame_id_lob`。

## 构建说明

默认根据架构选择 SDK 目录：

| 架构 | 目录 |
| ---- | ---- |
| x86_64 | `mindvisionSDK/lib/x64` |
| aarch64 | `mindvisionSDK/lib/arm64` |
| x86 | `mindvisionSDK/lib/x86` |
| arm | `mindvisionSDK/lib/arm` |

也可以手动指定：

```bash
colcon build --packages-select mindvision_camera --cmake-args -DMVSDK_ARCH=arm64
```

