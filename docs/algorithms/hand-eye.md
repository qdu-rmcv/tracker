# 手眼标定算法

## 标定目标

求解相机相对云台 pitch 轴的安装外参：

```text
camera -> gimbal
```

最终转换为可写入 xacro 或 launch 的：

```text
xyz:=tx ty tz
rpy:=roll pitch yaw
```

## 输入数据

每个样本包含：

| 数据 | 来源 |
| ---- | ---- |
| 棋盘格角点 | `/image_raw` |
| 相机内参 | `/camera_info` |
| 标定板相对相机位姿 | OpenCV `solvePnP` |
| 云台姿态 | LibXR `ahrs_quaternion` |

## 采集约束

节点拒绝以下样本：

- 没有检测到棋盘格
- 图像和 IMU 姿态时间差超过 `max_age_sec`
- 云台还在运动
- 清晰度低于 `check_min_blur_score`
- 与已有样本姿态距离小于 `check_min_angle_dist`

## 求解模式

| 参数 | 方法 | 说明 |
| ---- | ---- | ---- |
| `use_robot_world_handeye=true` | `calibrateRobotWorldHandEye` | 联合估计世界与手眼，推荐 |
| `use_robot_world_handeye=false` | `calibrateHandEye` | 经典 AX=XB |

经典模式的 `handeye_method` 支持：

- `TSAI`
- `PARK`
- `HORAUD`
- `ANDREFF`
- `DANIILIDIS`

## 结果解释

求解结果会包含：

- xacro 参数形式
- launch 传参形式
- 相机安装偏角参考值
- RobotWorld 模式下的标定板位姿估计

