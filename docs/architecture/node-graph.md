# 节点通信图

## 实车主链路

```mermaid
graph LR
    CAM["camera_node<br/>hik/mindvision"] -->|/image_raw| DET["armor_detector"]
    CAM -->|/camera_info| DET
    DET -->|/detector/armors| TRK["armor_tracker"]
    TRK -->|/tracker/target| TRAJ["planning_trajectory"]
    TRAJ -->|/trajectory/send| SER["serial_driver"]
    SER -->|/joint_states| RSP["robot_state_publisher"]
    RSP -->|/tf / /tf_static| DET
    RSP -->|/tf / /tf_static| TRK
    RSP -->|/tf / /tf_static| TRAJ
    TRK -->|/tracker/info| MARK["armor_marker"]
    TRAJ -->|/trajectory/info| MARK
    DET -->|/detector/debug_*| MARK
```

## 纯仿真链路

```mermaid
graph LR
    SIM["rm_simulator"] -->|/detector/armors| TRK["armor_tracker"]
    SIM -->|/tf_static| TRK
    SIM -->|/ground_truth/*| MARK["armor_marker"]
    TRK -->|/tracker/target| TRAJ["planning_trajectory"]
    TRAJ -->|/trajectory/send| NULL["无串口消费者"]
    TRAJ -->|/trajectory/info| MARK
```

## 视频回放链路

```mermaid
graph LR
    VID["video_publisher"] -->|/image_raw| DET["armor_detector"]
    VID -->|/camera_info| DET
    DET -->|/detector/armors| VIEW["查看识别结果或接入 tracker"]
    DET -->|/detector/result_img| VID
    VID -->|保存结果视频 可选| FILE["result_video_path"]
```

## 主要话题速查

| 话题 | 发布者 | 订阅者 | 说明 |
| ---- | ------ | ------ | ---- |
| `/image_raw` | 相机 / 视频 | `armor_detector`, `rm_hand_eye_calibrate` | RGB 图像 |
| `/camera_info` | 相机 / 视频 | `armor_detector`, `rm_hand_eye_calibrate` | 相机内参 |
| `/detector/armors` | `armor_detector` / `rm_simulator` | `armor_tracker`, `armor_marker` | 装甲板观测 |
| `/tracker/target` | `armor_tracker` | `planning_trajectory`, `armor_marker` | 跟踪目标 |
| `/trajectory/send` | `planning_trajectory` | `rm_serial_driver` | 云台目标角和开火 |
| `/joint_states` | `rm_serial_driver` | `robot_state_publisher` | 云台 yaw/pitch |
| `/tf`, `/tf_static` | `robot_state_publisher` / `rm_simulator` | 识别、跟踪、弹道 | 坐标变换 |
