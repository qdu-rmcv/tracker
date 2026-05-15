# 仿真模型

## 目标

`rm_simulator` 用参数化目标替代真实识别输出，让跟踪和弹道可以独立调试。

## 仿真输出

| 输出 | 内容 |
| ---- | ---- |
| `/detector/armors` | 带噪声且可见的装甲板，模拟 detector 输出 |
| `/ground_truth/armors` | 真值装甲板 |
| `/ground_truth/noisy_armors` | 带噪声真值 |
| `/ground_truth/robot_pose` | 车体中心真值 |

## 运动模型

仿真器维护目标初始位姿和速度：

- `robot.init_x/y/z/yaw`
- `robot.vx/vy/vz`
- `robot.omega`

当前代码中 X 方向速度使用一个余弦函数示例：

```text
vx(t) = pi / 2 * cos(pi / 2 * t)
```

如果需要常量速度，可以改回参数 `robot.vx`。

## 噪声模型

位置和姿态噪声使用线性距离相关模型：

```text
sigma = k0 + k1 * distance
```

对应参数：

- `noise.k0_pos`, `noise.k1_pos`
- `noise.k0_ori`, `noise.k1_ori`

