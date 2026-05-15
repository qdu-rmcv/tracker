# 视频回放流程

## 启动识别回放

```bash
source install/setup.bash
ros2 launch rm_vision_bringup video.launch.py video_path:=/path/to/video.mp4
```

## 覆盖播放参数

```bash
ros2 launch rm_vision_bringup video.launch.py \
  video_path:=/path/to/video.mp4 \
  loop:=true \
  playback_rate:=0.5 \
  publish_fps:=30.0
```

## 保存识别结果视频

在参数文件中配置：

```yaml
/video_publisher:
  ros__parameters:
    save_result_video: true
    result_img_topic: /detector/result_img
    result_video_path: /tmp/result.mp4
    result_video_fourcc: mp4v
```

确保 `armor_detector.debug=true`，否则不会发布 `/detector/result_img`。

## 调试重点

- 视频 frame_id 应与后续 TF 配置一致
- 回放内参应匹配视频来源相机
- `publish_fps` 过高会重复消耗计算，过低会影响跟踪连续性

