# 识别配置

## 后端选择

```yaml
/armor_detector:
  ros__parameters:
    detector_type: traditional
    debug: false
```

| 参数 | 可选值 | 说明 |
| ---- | ------ | ---- |
| `detector_type` | `traditional`, `yolo` | 检测后端 |
| `debug` | `true/false` | 是否发布调试输出 |

## 传统检测参数

```yaml
binary_lower_thres: 160
binary_upper_thres: 255
detect_color: 0
classifier_threshold: 0.7
ignore_classes: ["negative"]
```

| 参数组 | 参数 | 说明 |
| ------ | ---- | ---- |
| `light` | `min_ratio`, `max_ratio`, `max_angle` | 灯条筛选 |
| `armor` | `min_light_ratio` | 两灯条长度比例 |
| `armor` | `min/max_small_center_distance` | 小装甲板中心距离 |
| `armor` | `min/max_large_center_distance` | 大装甲板中心距离 |
| `armor` | `max_angle` | 装甲板倾角 |
| `corner_corrector` | `use_corner_corrector` 等 | 灯条角点修正 |

## YOLO 参数

```yaml
yolo:
  model_path: ""
  device: CPU
  input_size: 640
  score_threshold: 0.7
  min_confidence: 0.8
  nms_threshold: 0.3
  num_keypoints: 4
  large_armor_ratio_threshold: 3.2
  end_to_end: false
```

## PnP 与优化

```yaml
pnp_filter:
  use_new_pnp_filter_method: false
  max_normal_dot: 0.0
  reproj_weight: 1.0
  normal_weight: 0.5

optimizer:
  use_pose_optimizer: false
  optimize_method: RANGE_SHORT_LM
  standard_pitch_deg: 15.0
  outpost_pitch_deg: -15.0
```

启用 `optimizer.use_pose_optimizer` 后，识别节点会依赖 TF。

