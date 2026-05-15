# rm_gimbal_description - 云台描述包

## 概述

`rm_gimbal_description` 提供云台和相机的 URDF/Xacro 描述，用于 `robot_state_publisher` 发布 TF。

**路径**: `src/rm_gimbal_description/`

## 文件结构

```text
rm_gimbal_description/
├── urdf/
│   └── rm_gimbal.urdf.xacro
├── CMakeLists.txt
└── package.xml
```

## 坐标系

| 坐标系 | 说明 |
| ------ | ---- |
| `odom` | 外部惯性参考 |
| `gimbal_odom` | 云台 yaw 参考惯性系 |
| `yaw_link` | yaw 轴 |
| `pitch_link` | pitch 轴 |
| `camera_link` | 相机机械坐标系 |
| `camera_optical_frame` | 相机 optical 坐标系 |

## 参数来源

`rm_vision_bringup/launch/common.py` 调用 xacro 时传入：

| 参数 | 来源 |
| ---- | ---- |
| `xyz` | `launch_params.yaml` 中 `odom2camera.xyz` |
| `rpy` | `launch_params.yaml` 中 `odom2camera.rpy` |
| `lob_xyz` | `launch_params.yaml` 中 `odom2camera.lob_xyz` |
| `lob_rpy` | `launch_params.yaml` 中 `odom2camera.lob_rpy` |

## 维护原则

- `/joint_states` 的 joint name 必须与 URDF 中关节名一致。
- 相机外参建议先机械测量，再用手眼标定微调。
- 如果 PnP 距离正确但跟踪坐标方向异常，优先检查 optical frame 旋转和 URDF `rpy`。

