# hik_camera - 海康相机驱动包

## 概述

`hik_camera` 封装海康 MVS SDK，发布 `image_raw` 和 `camera_info`，并支持英雄机器人普通相机与吊射相机切换。

**路径**: `src/hik_camera/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | launch 中为 `camera_node`，类内默认 `hik_camera_node` |
| 组件插件 | `HikCamera::HikCameraNode` |
| 可执行文件 | `hik_camera_node` |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `image_raw` | `sensor_msgs/msg/Image` | 相对节点命名空间发布，常用为 `/image_raw` |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像同步发布 |
| `/camera_switch_done` | `std_msgs/msg/Bool` | 英雄机器人相机切换完成 |

## 订阅话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/lob_shot_switch` | `std_msgs/msg/Bool` | 英雄机器人触发普通/吊射相机切换 |

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `camera_name` | `gimbal_camera` | 普通相机名称 |
| `camera_info_url` | `package://hik_camera/config/camera_info.yaml` | 普通相机内参 |
| `device_index` | `0` | SDK 设备索引 |
| `frame_id` | `camera_optical_frame` | 图像坐标系 |
| `exposure_time` | `1000.0` | 曝光时间，单位 us |
| `gain` | `15.0` | 增益 |
| `autocap` | `true` | 自动取流相关开关 |
| `frame_rate_enable` | `false` | 是否启用固定帧率 |
| `frame_rate` | `249.0` | 固定帧率 |
| `grab_timeout_ms` | `20` | SDK 取流短超时 |
| `image_node_num` | `1` | SDK 缓存节点数 |
| `rotate` | `0` | `0/1/2/3` 对应不旋转、顺时针 90、180、逆时针 90 |

英雄机器人额外参数：

| 参数 | 说明 |
| ---- | ---- |
| `camera_name_lob` | 吊射相机名称 |
| `camera_info_url_lob` | 吊射相机内参 |
| `device_index_lob` | 吊射相机 SDK 设备索引 |
| `frame_id_lob` | 吊射相机坐标系 |

## 启动

```bash
ros2 launch hik_camera hik_camera.launch.py
```

## 注意事项

- 编译时会按 `x86_64` 或 `aarch64` 选择 `hikSDK/lib/amd64` 或 `hikSDK/lib/arm64`。
- SDK 取流短超时不会立即判为掉线，非短超时错误会触发守护线程重启相机。
- 输出编码为 `rgb8`。

