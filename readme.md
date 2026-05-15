# before all

获取源码：
```bash
git clone https://github.com/QDU-VRobot/vision_dev.git
```

第一次 `clone` 本仓库，则需配置 `hook`：
```bash
git config core.hooksPath .githooks
```

获取配置文件：
```bash
git submodule update --init --recursive
```

获取测试资源：

首先需要安装 git lfs
```bash
sudo apt install git-lfs
git lfs install
```

```bash
git clone git@github.com:QDU-VRobot/test_assets.git src/rm_vision/rm_vision_bringup/test_assets
cd src/rm_vision/rm_vision_bringup/test_assets
git lfs pull
```

要使用 `launch` 脚本，方法如下：
```bash
ros2 launch rm_vision_bringup vision_bringup.launch.py robot:=<robot_type>
```

#### 基本的工作流程：

`master` 为稳定分支，`dev` 为开发中分支。

将dev分支fork下来，进行自己负责的模块的开发/改动。

改动完成后首先将上游仓库的新commit都pull下来，如果有冲突则解决冲突，随后向上游仓库提交pr。

提交pr后进行review，此时也可在pr界面指定你期望的成员作为reviewer，会发邮件进行提醒。

确认没有问题后进行merge。master会在dev阶段性测试无误后merge。

# 简介

本项目源自于RV并做出了发展，识别器，预测器，弹道结算全放在了上位机，在机器人运动控制做好的情况下，一辆车的配适时间压缩到半小时，视觉从0上手调车只需要半天。

# 调车

### 1.相机标定

打开两个终端，source后分别输入：

```bash
ros2 launch hik_camera hik_camera.launch.py
```

```bash
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.02 image:=/image_raw camera:=/hik_camera
```

`size` 为棋盘格交点的数量 `width x height`

`square` 为一个黑色棋子的大小，单位为 m

标定完成后修改rm_vision/config/camera_info。

**检验标定结果**

PnP解算的距离与实际距离在7m,5m,3m,1.5m时的误差

### 2.机器人坐标系维护：

根据实际测量修改urdf文件，尽可能符合实际情况。通过可视化展示，看机器人建模是否正确

### 3.对齐时间戳

IMU与相机多传感修正：目标不动，本车动，看position的变化
如果position差值大，则证明时间戳没有对齐，这时应该调整time_offset，去尽量对齐时间戳。
IMU读值快，时间戳对齐采用线性插值法，用最接近图像数据的IMU的两帧时间戳，求和取平均得到的值去和图形数据配对。
时间戳对齐的目标不懂晃动车头，在不丢失目标的情况下动，position.x的波动范围为0.04以内。

### 4.卡尔曼滤波

计算方差：
识别装甲板，观察预测出的x，y，z，Yaw(target中的数据)，
首先静止不动等数据稳定，记录当前值(记为stable)
然后用手缓慢左右摆动枪管，但不要跟丢装甲板(保证卡尔曼滤波是连续的没有断掉)，
然后记录下极大值和极小值的差(记为diff)，然后按以下公式计算出方差

$$
\frac{(\frac {diff} {4})^2}{stable}
$$

### 5.打静止装甲板，修正到目标中心

在确定urdf文件描述正确的情况下，可以硬补。

### 6.不同距离下打旋转装甲板

### 7.自启动脚本 auto.sh

# 使用

### 安装ROS

  [Ubuntu22.04.1安装ROS2入门级教程(ros-humble)_ros humble_Python-AI Xenon的博客-CSDN博客](https://blog.csdn.net/yxn4065/article/details/127352587)

### 创建工作空间

```Shell
mkdir -p ~/AUTO_AIM
```

### 下载源代码

  在 `AUTO_AIM` 目录下

```Shell
git clone https://github.com/QDU-VRobot/vision_dev.git --recursive
cd vision_dev
git switch dev # 开发分支
git switch master # 稳定分支
```

```Shell
sudo apt install ros-humble-foxglove-bridge
```

### 编译

```Shell
rosdep install --from-paths src --ignore-src -r -y
```

```Shell
colcon build --symlink-install
```

### 运行节点

```Shell
sudo chmod 777 /dev/ttyACM0
```

  运行每个节点，必须新建终端并输入命令，且运行前需要执行 `source install/setup.bash`

```Shell
source install/setup.bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

```Shell
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

- 单独运行子模块

```Shell
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

```Shell
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

# 启动可视化

  打开新的终端

```Shell
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

# 一些新的修改

## 手眼标定

`rm_hand_eye_calibrate`包，编译好后使用以下命令启动：

```bash
ros2 launch rm_hand_eye_calibrate hand_eye_calibrate.launch.py
```

使用通过服务通信触发。

获取样本：

```bash
ros2 service call /rm_hand_eye/capture std_srvs/srv/Trigger "{}"
```

清空样本：

```bash
ros2 service call /rm_hand_eye/reset std_srvs/srv/Trigger "{}"
```

求解：

```bash
ros2 service call /rm_hand_eye/solve std_srvs/srv/Trigger "{}"
```

参数含义参考包目录下的 README.md。

## 将 IR 转换为 onnx / engine

注意：目前持有的 Orin 计算能力为 8.7，环境为 TensorRT 10.3、CUDA 12.6。个人电脑上配置环境注意尽量保持一致。

此版本 CUDA 最高支持的编译器版本为 gcc 13 / clang 17.

使用如下命令：

```bash
openvino2onnx yolo11.xml yolo11_raw.onnx -v 23
```

若为 sp 开源模型，使用如下命令：

```bash
openvino2onnx yolo11.xml yolo11_raw.onnx -u -v 23
```

即不进行静态检查。

随后运行 `fix_model` 脚本，修正模型中不兼容的部分，并验证模型是否正确。

```bash
python fix_model.py -i yolo11_raw.onnx -o yolo11_fixed.onnx
```

若对于其他模型有其他兼容性问题，则修改 `fix_model.py`，添加对应的修复逻辑。

于目标设备上使用如下命令将 onnx 转换为 engine：

```bash
trtexec --onnx=yolo11_fixed.onnx --saveEngine=yolo11.engine
```

如需转换时进行优化：

```bash
trtexec --onnx=yolo11_fixed.onnx --saveEngine=yolo11.engine --fp16 --builderOptimizationLevel=5 --timingCacheFile=trt_timing.cache --useSpinWait
```

可使用 `build_end2end`、`build_trt_engine` 脚本在 onnx 模型后插入 `EfficientNMS_TRT`，使其直接输出关键点、类别、置信度， 目前仅限 trt。