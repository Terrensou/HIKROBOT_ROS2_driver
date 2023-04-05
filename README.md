# HIKROBOT_ROS2_driver
基于ROS2 Humble和海康机器人工业相机Linux SDK 3.2.0写的简单驱动

## Build
``` bash
colcon build --packages-select hkcamera_driver
source install/setup.bash
```

## Usage
1. 启动图像取流（通过回调函数），发布至tpoic: /image_raw
```shell
ros2 run hik_camera_driver single_cam_node
```
2. 保存相机参数配置至cam_param_save.ini（每次相机断电后恢复默认值，可通过MVS界面设置后运行该方法导出配置。）
```shell
ros2 run hik_camera_driver single_cam_node
```
3. 导入相机参数配置：cam_param_load.ini
```shell
ros2 run hik_camera_driver single_cam_node
```
### To Do
- [ ] 启动相机前输出当前配置参数
- [ ] 支持py文件方式启动，自定义保存、加载参数文件地址
- [ ] 相机参数支持xml形式加载（过于遥远）
- [ ] 多相机节点
