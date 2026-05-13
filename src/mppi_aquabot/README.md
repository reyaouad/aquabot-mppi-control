# Aquabot Competition

This repository is the home to the source code and software documentation for the Aquabot Challenge. This challenge was organized by [Sirehna](https://www.sirehna.com) in 2024. It is based on the [VRX challenge](https://github.com/osrf/vrx).

The code in this repository is adapted for projects and labs and consists in multiple packages.

## Installation

The packages are compatible with the following duos:

- ROS 2 Galactic + Ignition Fortress (sea will be white due to a limitation of Ignition Rendering)
- ROS 2 Humble or Jazzy + Gazebo Garden or Harmonic

Set the `GZ_VERSION` environment variable to the Ignition / Gazebo version you pick.

### Dependencies

- For the base simulation:
  - ROS 2 and Gazebo bridge (`ros_ign_gazebo` on Galactic, `ros_gz_sim` on Garden+)
  - [`simple_launch`](https://github.com/oKermorgant/simple_launch), available with `apt` on Humble+
- For the helper packages:
  - `robot_localization`
  - `nav_msgs`

## Main simulation

The simulation is run through:
```
ros2 launch aquabot_gz turbines_launch.py world:={easy,medium,hard} (default easy)
```

This will run Gazebo with an autonomous boat with two steerable propellers and a pan camera. This boat has to inspect various wind turbines in a rocky region. Turbines have a QR code on one of their side. The robot is available in the `aquabot_description` package.

Several topics, for sensing and action, are available in the `/aquabot` namespace:

- sensor topics

 Topic        |   Message     | Description
:--------------------------:|:--------------------:|:-:
`sensors/cameras/main_camera_sensor/image_raw` | `sensor_msgs/Image` | simulated image
`sensors/imu/imu/data` | `sensor_msgs/Imu` | Boat IMU
`sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | Boat GPS
`ais_sensor/windturbines_positions` | `geometry_msgs/Pose` | GPS (lat-long) coord of the turbines
`sensors/acoustics/receiver/range`| `std_msgs/Float64` | Range to pinger
`sensors/acoustics/receiver/bearing`| `std_msgs/Float64` | Bearing to pinger

- actuation topics

 Topic                    |   Message          | Description
:--------------------------:|:--------------------:|:-:
`camera/cmd_pos`          | `std_msgs/Float64` | Camera angle
`thrusters/left/cmd_pos`  | `std_msgs/Float64` | Left thruster angle (+- pi/4)
`thrusters/right/cmd_pos` | `std_msgs/Float64` | Right thruster angle (+- pi/4)
`thrusters/left/thrust`   | `std_msgs/Float64` | Left thruster thrust (+- 5000)
`thrusters/right/thrust`  | `std_msgs/Float64` | Right thruster thrust (+- 5000)


When a QR code is read, its message should be published on the `/vrx/windturbinesinspection/windturbine_checkup` topic. When all QR codes have been read then the pinger will be moved near one of the turbines that needs to be inspected. The boat should stabilize for 30 sec in front of the corresponding QR code and then do a round turn of the turbine.

Compared to the initial challenge, all timers have been heavily increased so that users can try various missions in the simulator.


## Helper packages

### `aquabot_ekf`

This package contains a node (`gps2pose`) that handles all sensors except images. Its main use is to convert GPS-based information to metric one.

It suscribes to: `gps_fix`, `imu_raw`, and the pinger and windturbines position topics.

It publishes:

- `gps_xy` (`geometry_msgs/PoseStamped`): metric position based on the GPS info
- `imu` (`sensor_msgs/Imu`): IMU plus its covariance
- `/aquabot/turbines` (`geometry_msgs/PoseArray`): metric position of the windturbines
- The pinger position on `/tf` (`"pinger"` frame)

A configuration file to be used with `robot_localization` is also available, that can easily consume the topics published by `gps2pose` in order to get the robot estimate on `/tf` and `odom`.

This package is meant *not* to be ready-to-use, a launch file has to be written in order to use the node and EKF according to the available topics.

### `aquabot_motion`

This package contains two nodes to provide some tools for people not interested in this bits.

- Control node (`cmd.py`): This node subscribes to `cmd_vel` (`geometry_msgs/Twist`) and `odom` (`nav_msgs/Odometry`). It uses a simple PID to control the thrusters of the boat in order to track the desired twist.

- Planning node (`planner.py`): This node offers a `nav_msgs/GetPlan` service that find a path from two start and goal poses in the world frame. The path is returned through the service and is also published on the `plan` topic for visualization purpose. This node also subscribes to the `goal_pose` topic. In this case it will use the current pose of the boat as the start, the resulting path will be published on `plan`.
