import launch
import launch_ros.actions


def generate_launch_description():
    led_node = launch_ros.actions.Node(
        package='embedded_ros_marvin', 
        executable='led_driver',
        output='screen',
    )

    odrive_driver_node = launch_ros.actions.Node(
        package='embedded_ros_marvin',
        executable='odrive_driver',
        output='screen',
        remappings=[('enc_vel', 'enc_vel/raw')],
    )


    estop_driver_node = launch_ros.actions.Node(
        package='embedded_ros_marvin',
        executable='estop_driver',
        output='screen',
    )

    return launch.LaunchDescription([
        odrive_driver_node,
        led_node,
        estop_driver_node,
    ])
