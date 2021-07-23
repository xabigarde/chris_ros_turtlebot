chris_ros_turtlebot
===================

This meta-package handles the bring up of hardware and Gazebo-based simulation of the CNU specific configuration of Kobuki-based Turtlebot, with a Kinect sensor and YDLIDAR X4 lidar.

The repository includes three packages:  `chris_turtlebot_model`,  `chris_turtlebot_dashboard`, and `chris_turtlebot_bringup`.

To run the typical configuration of a single turtlebot in simulation:
```
roscore
roslaunch chris_world_models gazebo_empty_world.launch
roslaunch chris_turtlebot_bringup chris_turtlebot_gazebo.launch
```
Replace empty_world by the desired world model from the list at [CHRISLab World Models](https://github.com/CNURobotics/chris_world_models)

To bring up the equivalent setup on hardware, ```roslaunch chris_turtlebot_bringup chris_turtlebot_bringup.launch```.

Common optional launches consistent with either hardware or simulation include ```roslaunch chris_turtlebot_bringup chris_teleop_control.launch``` and ```roslaunch chris_turtlebot_bringup chris_turtlebot_rviz.launch```.

### chris_turtlebot_model

This package contains custom urdf files, meshes, and Gazebo-plugin for the Kobuki-based Turtlebot.

The files are designed to permit multiple instances of the robots in simulation for an multi-robot simulation.

The custom Gazebo plugin publishes ground truth pose for the base_footprint link that can be used with fake localization.

### chris_turtlebot_dashboard

This package contains a customized version of the kobuki_dashboard.  This version permits diagnostic messsages from within a particular robot's name space, and also handles stale messages for the battery indicators, as well as updated PyQt5 bindings


### chris_turtlebot_bringup

This package contains launch and parameter files for launching single and multi-robot instances in both hardware and simulation.  The system places each robot in a custom namespace with appropriate `tf_prefix` so that the robots can be tracked individually.

The package includes launch files that are compatible with the CNU Robotics [Flexible Navigation](https://github.com/CNURobotics/flexible_navigation) system using [CHRISLab Turtlebot Flexible Navigation](https://github.com/CNURobotics/chris_turtlebot_flexible_navigation).


| Hardware Only Launch Files | Description |
|-------------|-------------|
|chris_turtlebot_bringup.launch | This launches the robot base system on the turtlebot hardware within a given namespace (default: `turtlebot`).  |
|chris_turtlebot_node.launch| This launches the base platform hardware drivers and associated nodes within a given namespace (default: `turtlebot`).  |
|chris_ydlidar_bringup.launch | Launches YDLIDAR driver and associated hardware within robot namespace (called by `chris_turtlebot_bringup`) |
|chris_kinect_node.launch | Launches kinect sensor processing within robot namespace  (called by `chris_turtlebot_bringup`)|

| HW or Gazebo Launch Files | Description |
|-------------|-------------|
|chris_fake_localization.launch | Convert odometry message to /tf within namespace, which substitutes for a localization system, providing a subset of the ROS API used by `amcl`. |
|chris_teleop_control.launch | Launches the keyboard and joystick interfaces, as well as the command velocity mux or basic demo |
|chris_cmd_vel_mux.launch | This launches the command velocity mux (multiple input single output) configured for our setup |
|chris_joystick_teleop.launch | Joystick controller tied to command velocity mux within namespace |
|chris_keyboard_teleop.launch | Keyboard interface tied to command velocity mux within namespace |
|chris_turtlebot_rviz.launch | Launch RViz with predefined configuration based on configuration files defined for each robot namespace |

| Gazebo-Simulation Launch Files | Description |
|-------------|-------------|
|chris_turtlebot_gazebo.launch | This launches the robot base system in current gazebo simulation within a given namespace (default: `turtlebot`).  This launch is equivalent to `chris_turtlebot_bringup.launch` on hardware. |
|chris_gazebo_fake_localization.launch | Launches fake localization based on ground truth and provides a transform to world coordinates |
|gazebo_two_turtlebots.launch| Launch two turtlebots (`tbot0` and `tbot1`) into existing Gazebo simulation with optional fake localization and transform to world coordinates |
