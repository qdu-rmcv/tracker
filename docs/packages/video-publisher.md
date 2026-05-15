# video_publisher - 视频回放包

## 概述

`video_publisher` 从本地视频文件按时间线发布 ROS2 图像和相机内参，便于离线复现识别问题。它还可以订阅识别结果图像并保存为结果视频。

**路径**: `src/video_publisher/`

## 节点

| 项 | 内容 |
| -- | ---- |
| 节点名 | `video_publisher` |
| 组件插件 | `VideoPublisherNode` |
| 可执行文件 | `video_publisher_node` |

## 发布话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `image_raw` | `sensor_msgs/msg/Image` | 视频帧转换后的 RGB 图像 |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |

## 可选订阅

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/result_img` | `sensor_msgs/msg/Image` | 保存识别结果视频时订阅，话题可配置 |

## 主要参数

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `video_path` | `""` | 视频文件路径，必填 |
| `frame_id` | `camera_optical_frame` | 输出图像 frame |
| `camera_info_url` | `package://video_publisher/config/camera_info.yaml` | 内参文件 |
| `loop` | `true` | 是否循环播放 |
| `use_video_fps` | `true` | 是否优先使用视频元数据帧率 |
| `playback_rate` | `1.0` | 播放速率倍率 |
| `publish_fps` | `30.0` | 发布帧率，`0` 可进入自动模式 |
| `save_result_video` | `false` | 是否保存结果视频 |
| `result_img_topic` | `/result_img` | 结果图像输入话题 |
| `result_video_path` | `""` | 输出视频路径 |
| `result_video_fourcc` | `mp4v` | OpenCV VideoWriter FourCC |

## 启动

```bash
ros2 launch rm_vision_bringup video.launch.py video_path:=/path/to/video.mp4
```

