import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to the source package
    package_src_dir = os.path.join(os.path.expanduser("~"), "ros2_ws/src/embedded_ros_marvin")

    # Path to the parameter file in the source folder
    ekf_params_file = os.path.join(package_src_dir, "params", "arv_ekf.yaml")

    # Debugging: Print the YAML file path to ensure it's correct
    print(f"Loading EKF parameters from: {ekf_params_file}")

    # Define the EKF node
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file]
    )

    return LaunchDescription([
        ekf_node
    ])

