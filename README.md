# Aquabot MPPI Control

ROS 2 autonomous marine robot simulation using Model Predictive Path Integral control for path following and goal navigation.

## Project Goal

The goal of this project is to control an autonomous boat in simulation using an MPPI controller. The boat receives odometry, follows a reference path or RViz goal, and publishes thrust and steering commands to the left and right thrusters.

## Features

- ROS 2 C++ controller node
- MPPI-based trajectory optimization
- Odometry feedback
- RViz goal input
- Reference path tracking
- Left and right thruster force control
- Thruster angle control
- Dynamic prediction model for marine robot motion
- Smooth control and rate limiting
- Goal stopping condition

## Tech Stack

- ROS 2
- C++
- Eigen
- Gazebo / Ignition
- RViz
- Marine robotics
- Model Predictive Path Integral Control

## System Architecture

Main inputs:

- `/aquabot/odom`
- `/aquabot/goal_pose`
- `/aquabot/reference_path`

Main outputs:

- `/aquabot/thrusters/left/thrust`
- `/aquabot/thrusters/right/thrust`
- `/aquabot/thrusters/left/cmd_pos`
- `/aquabot/thrusters/right/cmd_pos`

## Controller Overview

The controller samples multiple control sequences, predicts the future state of the boat, evaluates each rollout using a cost function, and updates the nominal command sequence using weighted MPPI optimization.

The cost function includes:

- Longitudinal tracking error
- Lateral tracking error
- Yaw error
- Sway velocity penalty
- Yaw rate penalty
- Thrust effort penalty
- Steering angle penalty
- Smoothness penalty
- Near-goal stopping penalty

## How to Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
