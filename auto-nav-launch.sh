#!/bin/bash

# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Build (skip zed_aruco_localization)
gnome-terminal -- bash -c "colcon build --packages-skip zed_aruco_localization; echo 'Build finished. Press Enter to close.'; exec bash"

sleep 2  # give some time between opening terminals

# Teleop
gnome-terminal -- bash -c "ros2 launch marvin_bot_description teleop_launch.py; echo 'Teleop stopped. Press Enter to close.'; exec bash"

# CV: ZED camera
gnome-terminal -- bash -c \"ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i; echo 'ZED camera stopped. Press Enter to close.'; exec bash\"

# CV: Drivable area
gnome-terminal -- bash -c \"ros2 launch drivable_area drivable_area.launch.py; echo 'Drivable area stopped. Press Enter to close.'; exec bash\"

# Embedded: launch_embedded
gnome-terminal -- bash -c \"ros2 launch embedded_ros_marvin launch_embedded.py use_enc_odom:=true; echo 'Embedded launch stopped. Press Enter to close.'; exec bash\"

# Embedded: pure_pursuit_lookahead
gnome-terminal -- bash -c \"ros2 run embedded_ros_marvin pure_pursuit_lookahead; echo 'Pure pursuit stopped. Press Enter to close.'; exec bash\"

# Navigation: inflation_node
gnome-terminal -- bash -c \"ros2 run occupancy_grid_inflation inflation_node; echo 'Inflation node stopped. Press Enter to close.'; exec bash\"

# Navigation: planner_server
gnome-terminal -- bash -c \"ros2 launch planner_server planner_server.launch.py; echo 'Planner server stopped. Press Enter to close.'; exec bash\"

# Navigation: goal_selection
gnome-terminal -- bash -c \"ros2 run goal_selection goal_selection_node; echo 'Goal selection stopped. Press Enter to close.'; exec bash\"
