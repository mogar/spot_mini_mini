# ROS2 SpotMicro

This is a ROS2 workspace with a port of the original spot-mini-mini code from ROS1. It's intended for use with ROS2/rolling

To run it, navigate your terminal to this directory and do:

```
source /opt/ros/rolling/setup.bash 
colcon build
source install/setup.bash
ros2 launch mini_ros2 spot_move.launch 
```

You should now be able to run the gamepad pybullet sim from the original ROS1 repo.

## ToDo

1. finish porting all the other launch files and python nodes
2. make it easier to change the robot model