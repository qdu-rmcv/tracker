# libxr - 通信基础库 wrapper

## 概述

`libxr` 包把仓库内的 LibXR 作为 ROS2 依赖导出，供 `rm_serial_driver` 和 `rm_hand_eye_calibrate` 使用。

**路径**: `src/libxr/`

## 构建方式

外层 `CMakeLists.txt` 固定：

| 选项 | 值 | 说明 |
| ---- | -- | ---- |
| `LIBXR_SHARED_BUILD` | `ON` | 构建动态库 |
| `LIBXR_SYSTEM` | `linux` | Linux 系统层 |
| `LIBXR_DRIVER` | `linux` | Linux 驱动层 |

并导出：

- `libxr::xr`
- 安装后的 `include/libxr`
- `Eigen3` 依赖

## 在本项目中的用途

| 使用方 | 用途 |
| ------ | ---- |
| `rm_serial_driver` | UART 通信、Topic、Terminal、RamFS |
| `rm_hand_eye_calibrate` | 标定时读取下位机 IMU 四元数 |

## 注意事项

- `libxr` 本身包含大量平台代码，本项目实际使用 Linux system/driver。
- 如果构建时提示缺少 `libudev`、`wpa_client`、`libnm`，需要安装对应开发包。
- 不建议在视觉业务包中直接绕过 `rm_serial_driver` 使用 LibXR，避免通信协议分散。

