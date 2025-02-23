# Differential Drive Robot Controller with Waypoint Navigation

## Overview
This project implements a ROS 2-based differential drive robot controller that computes wheel RPM from velocity commands and a waypoint navigation system using a PID controller.

## Features
- Computes wheel RPM from `/cmd_vel` commands.
- Publishes RPM values for motor control.
- Uses a PID controller to navigate between waypoints.
- Compatible with ROS 2 Humble and Gazebo Harmonic.

## Installation
```sh
colcon build --packages-select car_nav2
source install/setup.bash
```

## Usage
1. **Run the RPM Publisher:**
   ```sh
   ros2 run car_nav2 rpm_publisher
   ```
2. **Run the Waypoint Navigator:**
   ```sh
   ros2 run car_nav2 waypoint_navigation.py --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
   ```

## Topics
- `/cmd_vel` (Input): Velocity commands.
- `/left_wheel_rpm`, `/right_wheel_rpm` (Output): Computed RPM values.
- `/odom` (Input): Odometry data for waypoint navigation.

## Parameters
- `wheelbase`: Distance between wheels.
- `wheel_radius`: Wheel radius.
- `max_rpm`: Maximum RPM limit.
- `kp`, `ki`, `kd`: PID controller gains.

## Testing
```sh
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
ros2 topic echo /left_wheel_rpm
ros2 topic echo /right_wheel_rpm
```
# car_nav2
Using ROS2 Jazzy and Gazebo Harmonic for autonomous navigation simulation of a robot car.

This project is based on the BME MOGI - ROS course, with the camera part removed, and the mapping and navigation features retained. For further learning, please refer to https://github.com/MOGI-ROS.

This project is for personal learning purposes only. If there are any copyright infringements, please contact us for removal.

You need to do this:

1.Run the following command to clone the car_nav2 repository to your local machine:

    git clone https://github.com/2024828/car_nav2.git

2.The file uses some Gazebo models, which you can download from:https://drive.google.com/file/d/1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_/view.

If you want to learn more about Gazebo models, please visit:https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics.

Make sure to let Gazebo know about their location by running:

    export GZ_SIM_RESOURCE_PATH=~/gazebo_models

3.Build the package and run:
    
    colcon build
    . install/setup.bash
    ros2 launch car_nav2 spawn_robot.launch.py

Open a new terminal and run:
    
    export GZ_SIM_RESOURCE_PATH=~/gazebo_models
    . install/setup.bash
    ros2 launch car_nav2 navigation_with_slam.launch.py

You can use the `2D goal pose` in the RViz2 toolbar to control the robot's movement and mapping. The robot will automatically plan the path.

You can also click the `+` on rviz2 to add the `NAV2 Goal` tool. Then, click on the `Waypoint/NavThrough Poses Mode` under the Nav2 plugin. You can use the `Nav2 Goal` to set multiple waypoints in sequence. By clicking the `StartWaypoint Following` at the bottom, you can start waypoint navigation, and the robot will move towards the preset target locations in order.
