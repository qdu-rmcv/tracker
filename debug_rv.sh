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

function build() {
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
}

function no_hardware(){
    ros2 launch rm_vision_bringup no_hardware.launch.py
}

function rm_vision_bringup(){
    devices=$(ls /dev/ttyACM[0-9]* /dev/ttyUSB[0-9]* 2>/dev/null)   # 隐藏报错
    if [ -z "$devices" ]; then
        echo "未找到任何串口设备"
        exit 1
    fi

    echo "$devices"

    sudo chmod 777 /dev/tty*
    sleep 2
    ros2 launch rm_vision_bringup vision_bringup.launch.py
}

function foxglove(){
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
}

function camera_calibration(){
    gnome-terminal  -- bash -c "ros2 launch hik_camera hik_camera.launch.py; exec bash"
    sleep 0.5
    ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.02 image:=/image_raw camera:=/camera
}

# 主程序
main() {
    INFO

    sleep 1

    # 检查参数数量
    if [ "$#" -lt 1 ]; then
        echo "用法: $0 build || no_hardware || rm_vision_bringup || foxglove || calibration"
        exit 1
    fi

    # 处理参数
    case "$1" in
        b)
            build
            ;;
        n)
            no_hardware
            ;;
        r)
            rm_vision_bringup
            ;;
        f)
            foxglove
            ;;
        c)
            camera_calibration
            ;;
        *)
            echo "无效的参数: $1"
            echo "用法: $0 build || no_hardware || rm_vision_bringup || foxglove || calibration"
            exit 1
            ;;
    esac
}

# 调用主程序
main "$@"
