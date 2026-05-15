# 相机配置

## 通用参数

海康和 MindVision 驱动的大部分参数一致：

| 参数 | 说明 |
| ---- | ---- |
| `camera_name` | 相机内参管理器名称 |
| `camera_info_url` | 普通相机内参文件 |
| `device_index` | 相机索引 |
| `frame_id` | 图像 frame |
| `exposure_time` | 曝光时间 |
| `gain` | 增益 |
| `frame_rate_enable` | 是否启用固定帧率 |
| `frame_rate` | 固定帧率 |
| `grab_timeout_ms` | SDK 取流超时时间 |
| `image_node_num` | SDK 缓存节点数 |
| `fps_stat_period` | FPS 统计打印周期 |
| `rotate` | 图像旋转 |

## 海康示例

```yaml
/camera_node:
  ros__parameters:
    camera_name: gimbal_camera
    camera_info_url: package://hik_camera/config/camera_info.yaml
    device_index: 0
    frame_id: camera_optical_frame
    exposure_time: 2000.0
    gain: 0.0
    frame_rate_enable: false
    frame_rate: 249.0
    grab_timeout_ms: 20
    image_node_num: 1
    rotate: 2
```

## MindVision 额外参数

| 参数 | 说明 |
| ---- | ---- |
| `gamma` | SDK gamma 值 |
| `device_sn` | 设备序列号，空字符串表示只用索引 |
| `device_sn_lob` | 吊射相机序列号 |

## 英雄吊射相机

`robot_type=hero` 时启用：

```yaml
camera_name_lob: gimbal_camera_lob
camera_info_url_lob: package://.../camera_info_lob.yaml
device_index_lob: 1
frame_id_lob: camera_optical_frame_lob
```

`rm_serial_driver` 发布 `/lob_shot_switch` 后，相机节点会在普通相机和吊射相机之间切换。

