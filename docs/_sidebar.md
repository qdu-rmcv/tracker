- **首页**
  - [项目简介](/)

- **快速入门**
  - [环境依赖](guide/prerequisites.md)
  - [编译构建](guide/build.md)
  - [启动方法](guide/launch.md)
  - [标定入口](guide/calibration.md)

- **系统架构**
  - [架构总览](architecture/README.md)
  - [功能包全景](architecture/package-overview.md)
  - [节点通信图](architecture/node-graph.md)
  - [数据流](architecture/data-flow.md)
  - [坐标系与 TF](architecture/coordinate-system.md)
  - [运行模式](architecture/deployment-modes.md)

- **功能包详解**
  - [总览](packages/README.md)
  - [auto_aim_interfaces 接口定义](packages/auto-aim-interfaces.md)
  - [armor_detector 装甲板识别](packages/armor-detector.md)
  - [armor_tracker 目标跟踪](packages/armor-tracker.md)
  - [planning_trajectory 弹道解算](packages/planning-trajectory.md)
  - [armor_marker 可视化](packages/armor-marker.md)
  - [hik_camera 海康相机](packages/hik-camera.md)
  - [mindvision_camera 迈德威视相机](packages/mindvision-camera.md)
  - [rm_serial_driver 串口通信](packages/rm-serial-driver.md)
  - [rm_simulator 目标仿真](packages/rm-simulator.md)
  - [rm_gimbal_description 云台模型](packages/rm-gimbal-description.md)
  - [rm_hand_eye_calibrate 手眼标定](packages/rm-hand-eye-calibrate.md)
  - [video_publisher 视频回放](packages/video-publisher.md)
  - [rm_vision_bringup 启动编排](packages/rm-vision-bringup.md)
  - [libxr 通信基础库](packages/libxr.md)

- **接口参考**
  - [话题列表](interfaces/topics.md)
  - [消息定义](interfaces/messages.md)
  - [服务列表](interfaces/services.md)

- **算法说明**
  - [算法概览](algorithms/README.md)
  - [装甲板识别](algorithms/armor-detection.md)
  - [目标跟踪](algorithms/tracking.md)
  - [弹道解算](algorithms/trajectory.md)
  - [手眼标定](algorithms/hand-eye.md)
  - [仿真模型](algorithms/simulator.md)

- **配置指南**
  - [配置总览](configuration/README.md)
  - [相机配置](configuration/camera-config.md)
  - [识别配置](configuration/detector-config.md)
  - [跟踪配置](configuration/tracker-config.md)
  - [弹道配置](configuration/trajectory-config.md)
  - [串口配置](configuration/serial-config.md)
  - [Launch 配置](configuration/launch-config.md)

- **工作流程**
  - [实车运行流程](workflow/live-run.md)
  - [调车流程](workflow/tuning.md)
  - [相机与手眼标定](workflow/calibration-workflow.md)
  - [纯仿真流程](workflow/simulation.md)
  - [视频回放流程](workflow/video-replay.md)

- **诊断**
  - [常见问题](troubleshooting/README.md)
