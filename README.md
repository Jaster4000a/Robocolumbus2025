# Robocolumbus2025

## 0) General Information
    - Computer: Nvidia Jetson Orin NX
    - Firmware: Jetpack 6.0
    - GPU: 15.3 GB (16GB)

## 1) Before running anything, you need to source the ros environment
    source /opt/ros/humble/setup.bash

## 2) Common Commands
### ros2 topic commands:
    ros2 topic list (lists out topic)
    ros2 topic echo <TOPIC NAME> (prints topic stream)
    ros2 topic info <TOPIC NAME> (tells you about data type)
        
### Gazebo Simulator with Clearpath robot: 
#### Source: https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install/
#### Load Simulator World
    ros2 launch clearpath_gz simulation.launch.py world:=simple_baylands x:=10 yaw:=1.5707
#### Load Simulator World with rviz
    ros2 launch clearpath_gz simulation.launch.py world:=simple_baylands x:=10 yaw:=1.5707 rviz:=True
#### Intel Realsense D405 Camera:
#### Source: https://github.com/IntelRealSense/realsense-ros
#### Load intel realsense viewer (--debug is optional)
    realsense-viewer --debug
#### Publish camera topics (ros wrapper)
    ros2 run realsense2_camera realsense2_camera_node
    
