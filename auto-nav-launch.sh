#!/bin/bash

# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Build (skip zed_aruco_localization)
gnome-terminal -- bash -c "colcon build --packages-skip zed_aruco_localization"

sleep 5  # give some time between opening terminals

# Embedded: launch_embedded
gnome-terminal -- bash -c "source ~/.bashrc; ros2 launch embedded_ros_marvin launch_embedded.py use_enc_odom:=true"
gnome-terminal -- bash -c "source ~/.bashrc; ros2 run embedded_ros_marvin pure_pursuit_lookahead"

# CV: ZED camera and Drivable area
gnome-terminal -- bash -c "source ~/.bashrc; ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i"
gnome-terminal -- bash -c "source ~/.bashrc; ros2 launch drivable_area drivable_area.launch.py"

# # Navigation: inflation_node | planner_server | goal_selection
gnome-terminal -- bash -c "source ~/.bashrc; ros2 run occupancy_grid_inflation inflation_node"
gnome-terminal -- bash -c "source ~/.bashrc; ros2 launch planner_server planner_server.launch.py"
gnome-terminal -- bash -c "source ~/.bashrc; ros2 run goal_selection goal_selection_node"
