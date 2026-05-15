# 架构总览

## 分层视角

```mermaid
graph TB
    subgraph IO["硬件与数据源"]
      CAM["工业相机 / 视频"]
      MCU["电控下位机"]
      SIM["rm_simulator"]
    end

    subgraph Perception["感知层"]
      DET["armor_detector<br/>装甲板识别 + PnP"]
    end

    subgraph State["状态层"]
      TRK["armor_tracker<br/>多模型 EKF 跟踪"]
      TF["robot_state_publisher<br/>TF 树维护"]
    end

    subgraph Control["解算与控制输出"]
      TRAJ["planning_trajectory<br/>弹道解算"]
      SER["rm_serial_driver<br/>LibXR 串口桥接"]
    end

    subgraph Tools["工具层"]
      MARK["armor_marker"]
      CALIB["rm_hand_eye_calibrate"]
    end

    CAM --> DET
    SIM --> TRK
    DET --> TRK
    MCU --> SER
    SER --> TF
    TF --> DET
    TF --> TRK
    TF --> TRAJ
    TRK --> TRAJ
    TRAJ --> SER
    DET --> MARK
    TRK --> MARK
    TRAJ --> MARK
    CAM --> CALIB
    MCU --> CALIB
```

## 关键设计点

- 主链路节点都以 component 形式注册，`rm_vision_bringup` 可把多个节点装进同一个 `component_container_mt`。
- `planning_trajectory` 独立容器运行，便于配置独立线程数、CPU 亲和性和实时线程。
- 识别输出位于相机光学坐标系，跟踪节点通过 TF 转换到 `target_frame`，默认是 `odom`。
- 串口节点是 ROS2 与 LibXR Topic 的边界，ROS2 侧使用 `/trajectory/send`，下位机侧使用 `target_euler`、`fire_notify`、`target_num` 等 LibXR 话题。
- `rm_simulator` 可以替代识别节点直接发布 `/detector/armors`，用于无相机调试。

## 主要数据闭环

| 阶段 | 输入 | 输出 | 说明 |
| ---- | ---- | ---- | ---- |
| 采集 | 相机 SDK / 视频文件 | `/image_raw`, `/camera_info` | 图像和内参 |
| 识别 | 图像、内参、可选 TF | `/detector/armors` | 装甲板类别和相机系位姿 |
| 跟踪 | `/detector/armors`, TF | `/tracker/target` | 目标中心、速度、yaw、半径 |
| 解算 | `/tracker/target`, TF | `/trajectory/send` | pitch/yaw/开火/目标编号 |
| 通信 | `/trajectory/send` | LibXR UART | 控制下发，下位机回传云台姿态 |
| TF | `/joint_states`, URDF | `/tf`, `/tf_static` | 提供云台与相机坐标关系 |

