# rm_auto_record

## Overview

`rm_auto_record` is a ROS2 package that records the custom topics automatically. It will subscribe the control topic `record_controller` to start or stop the recording automatically. 

## Quick Start

8   
```bash
# Clone the repository
cd <path to your workspace>/src
git clone https://github.com/FaterYU/rm_auto_record.git

# Build the package
cd <path to your workspace>
colcon build --symlink-install --packages-select rm_auto_record

# Source the workspace
source install/setup.bash

# Run the node
ros2 launch rm_auto_record auto_record.launch.py
```

## Modify

1. Make sure your topic name and type, and edit the config file in `config/node_params.yaml` as follows:

    ```yaml
    /rm_auto_record:
    ros__parameters:
        topic_name:
        - "/image_raw/compressed"
        - "/camera_info"
        topic_type:
        - "sensor_msgs/msg/CompressedImage"
        - "sensor_msgs/msg/CameraInfo"
    ```

    Remember to keep the same order of topic name and type.

 2. Modify the source code in `include/rm_auto_record.hpp` to include the message header file in the scope begining with the comment `// Include the message header file here`. The example is as follows:

    ```cpp
    ...
    // Include the message header file here
    #include "sensor_msgs/msg/compressed_image.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    // end of the inclusion
    ...
    ```

3. Modify the source code in `include/rm_auto_record.hpp` to declare the message subscriber type in the scope begining with the comment `// Declare the message subscriber type here`. The example is as follows:

    ```cpp
    ...
    // Declare the message subscriber type here
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    // end of the declaration
    ...
    ```

4. Modify the source code in `src/rm_auto_record.cpp` to initialize the message subscriber in the scope begining with the comment `// Initialize the message subscriber here`. The example is as follows:

    ```cpp
    ...
    // Initialize the message subscriber here
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic_name[0], 10, std::bind(&RmAutoRecord::image_callback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic_name[1], 10, std::bind(&RmAutoRecord::camera_info_callback, this, std::placeholders::_1));
    // end of the initialization
    ...
    ```

    Remember to keep the same order of parameter in `config/node_params.yaml` and the order of initialization.

5. Don't forget to add your dependencies in `package.xml`.
6. (Optional) Modify the launch file in `config/node_params.yaml` to change the save path variable `uri` using the absolute path. The default save path is `/ros_ws/`.
