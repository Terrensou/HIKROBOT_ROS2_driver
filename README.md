# HIKROBOT_ROS2_driver
基于ROS2 Humble和海康机器人Linux SDK 3.2.0写的简单驱动

``` bash
colcon build --packages-select hkcamera_driver
source install/setup.bash
ros2 run hik_camera_driver single_cam_node
```