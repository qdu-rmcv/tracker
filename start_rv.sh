#!/bin/bash

# 函数定义

function INFO(){

    source install/setup.bash
    source /opt/ros/humble/setup.bash
    
    echo "======================="
    echo "Welcome to use the rv!"
    echo "======================="
    echo ""
    echo "请检查 一下事项"
    echo "1. 相机线是否连接"
    echo "2. 串口线是否连接"
    echo "3. camera_info使用的是哪个相机"
    echo "4. 电控代码是否烧得是这一套"
    echo "5. 电控给速度是否正常"
    echo ""
    echo "早睡觉,莫熬夜.通宵会降低智力 :( "
    echo ""
}

function no_hardware(){
    ros2 launch rm_vision_bringup no_hardware.launch.py
}

function rm_vision_bringup(){
    ls /dev/tty*
    sudo chmod 777 /dev/tty*
    sleep 2
    ros2 launch rm_vision_bringup vision_bringup.launch.py
}

# 主程序
main() {
    INFO
    
    rm_vision_bringup
}

# 调用主程序
main "$@"
