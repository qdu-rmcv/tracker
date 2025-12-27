# 简介
本项目源自于RV并做出了发展，识别器，预测器，弹道结算全放在了上位机，在机器人运动控制做好的情况下，一辆车的配适时间压缩到半小时，视觉从0上手调车只需要半天。

# 调车
### 1.相机标定
```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/camera/image camera:=/camera
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
时间戳对齐的目标不懂晃动车头，在不丢失目标的情况下后opsition.x的波动范围为0.04以内。
### 4.卡尔曼滤波
计算方差：
识别装甲板，观察预测出的x，y，z，Yaw(target中的数据)，
首先静止不动等数据稳定，记录当前值(稳态值)
然后用手缓慢左右摆动枪管，但不要跟丢装甲板(保证卡尔曼滤波是连续的没有断掉)，
然后记录下极大值和极小值的差(这里称之为区间)，然后按以下公式计算出方差
（区间/4）^2/稳态值

### 5.打静止装甲板，修正到目标中心
在确定urdf文件描述正确的情况下，可以硬补。
### 6.不同距离下打旋转装甲板
### 7.自启动脚本 auto.sh

### 注意事项：
1.识别器部分，目前主要影响因素是神经网络识别数字部分，灯条识别有足够的稳定性，但数字识别受环境光影响较大，调车与改进代码时重点关注。   

2.当目标车装甲板安装并不严格遵守中心对称时，增大sigma2_q_r噪声参数。

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
git clone git@github.com:MzKyle/Observation-of-the-whole-vehicle.git
```


```Shell
sudo apt install ros-humble-foxglove-bridge
```


### 编译

  在 `AUTO_AIM` 目录下

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
ros2 launch auto_aim_bringup auto_aim.launch.py 
```


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


