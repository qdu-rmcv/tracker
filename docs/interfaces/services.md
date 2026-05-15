# 服务列表

## 手眼标定服务

`rm_hand_eye_calibrate` 提供三个服务：

| 服务 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/rm_hand_eye/capture` | `std_srvs/srv/Trigger` | 捕获当前标定样本 |
| `/rm_hand_eye/reset` | `std_srvs/srv/Trigger` | 清空全部样本 |
| `/rm_hand_eye/solve` | `std_srvs/srv/Trigger` | 求解手眼外参 |

调用示例：

```bash
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

## 当前主链路

识别、跟踪、弹道、串口主链路当前主要通过话题通信，不提供控制服务。启停、模式切换和参数调整主要通过 launch 参数、ROS 参数和节点重启完成。

## 扩展建议

如果后续需要运行时控制，可以优先考虑：

| 服务 | 建议类型 | 作用 |
| ---- | -------- | ---- |
| `/detector/set_debug` | `std_srvs/SetBool` | 动态开启或关闭调试输出 |
| `/trajectory/reset` | `std_srvs/Trigger` | 清空弹道内部状态 |
| `/tracker/reset` | `std_srvs/Trigger` | 强制 tracker 回到 LOST |

新增服务时需要同步文档中的话题/服务契约。

