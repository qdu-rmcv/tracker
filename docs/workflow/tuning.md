# 调车流程

## 1. 相机和曝光

目标：

- 灯条不过曝到完全连片
- 数字区域仍可分类
- 图像帧率稳定

先调：

- `exposure_time`
- `gain`
- `gamma`，仅 MindVision
- `rotate`

## 2. 相机内参

用 `camera_calibration` 标定并写入 `camera_info.yaml`。验证 PnP 距离时建议在多个距离点检查，例如 1.5m、3m、5m、7m。

## 3. URDF 与手眼

先测量相机到云台 pitch 轴的机械外参，写入 `launch_params.yaml`。再用 `rm_hand_eye_calibrate` 优化 `xyz` 和 `rpy`。

## 4. 时间戳

固定目标不动，晃动车头。如果 tracker 输出位置随车体运动明显漂移，优先调整 `timestamp_offset`。

## 5. 识别参数

传统识别先调：

- `binary_lower_thres`
- `light.*`
- `armor.*`
- `classifier_threshold`

打开 debug 后看：

- `/detector/binary_img`
- `/detector/number_img`
- `/detector/result_img`

## 6. 跟踪参数

识别稳定后再调 tracker：

- 丢目标频繁：检查 `max_match_distance`、`lost_time_thres`
- 切换迟钝：检查 `change_time_thres`
- 旋转目标中心抖：检查 EKF 过程噪声和半径约束

## 7. 弹道参数

最后调弹道：

- `bias_time`
- `s_bias`
- `z_bias`
- `pitch_bias`
- 弹道表
- 开火条件相关逻辑

