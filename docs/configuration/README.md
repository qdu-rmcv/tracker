# 配置总览

项目配置来源分为三类：

| 类型 | 位置 | 说明 |
| ---- | ---- | ---- |
| 包内默认配置 | 各包 `config/` | 相机默认内参、串口默认 VID/PID |
| 机器人运行配置 | `rm_vision_bringup/config/` | 实车节点参数、弹道表、launch 参数 |
| 代码默认参数 | `declare_parameter` | YAML 未覆盖时使用 |

## 推荐配置文件结构

`rm_vision_bringup/config/` 通常应包含：

```text
config/
├── launch_params.yaml
├── node_params.yaml
├── camera_info.yaml
├── camera_info_lob.yaml
├── table.bin
└── table_lob.bin
```

`launch_params.yaml` 管编排和外参，`node_params.yaml` 管节点参数。

## 参数覆盖顺序

ROS2 参数通常按以下顺序生效：

1. 代码中的 `declare_parameter` 默认值
2. YAML 文件
3. launch 中额外传入的字典参数
4. 命令行参数覆盖

