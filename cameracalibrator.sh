#!/bin/bash

delay_time=3

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 开第一个终端运行相机
gnome-terminal -- bash -c "cd \"$SCRIPT_DIR\" && source install/setup.bash && ros2 launch hik_camera hik_camera.launch.py"

# 等待2秒让相机启动
sleep 2

# 开第二个终端运行标定
gnome-terminal -- bash -c "cd \"$SCRIPT_DIR\" && source install/setup.bash && ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.02 image:=/image_raw camera:=/hik_camera; exec bash"

echo "两个终端已打开！"

while true
do
    ps -ef | grep "hik_camera_node" | grep -v "grep" > /dev/null
    
    if [ $? -ne 0 ]
    then
        echo "[$(date '+%H:%M:%S')] camera未运行"
               
        echo "等待 ${delay_time}秒 后启动..."
        sleep $delay_time
        
        gnome-terminal -- bash -c "cd \"$SCRIPT_DIR\" && source install/setup.bash && ros2 launch hik_camera hik_camera.launch.py"
        
        [ $? -eq 0 ] && echo "启动成功" || echo "启动失败"
    else
        echo "[$(date '+%H:%M:%S')] camera running"
    fi
    
    sleep $delay_time
done
