import launch
import launch_ros.actions


def generate_launch_description():
    led_node = launch_ros.actions.Node(
        package='embedded_ros_marvin', 
        executable='LED_subscriber',
        output='screen',
    )

    dual_odrive_controller_node = launch_ros.actions.Node(
        package='embedded_ros_marvin', 
        executable='dual_odrive_controller',
        output='screen',
        remappings=[('enc_vel', 'enc_vel/raw')],
    )


    serial_estop_monitor_node = launch_ros.actions.Node(
        package='embedded_ros_marvin',
        executable='serial_estop_monitor',
        output='screen',
    )

    return launch.LaunchDescription([
        dual_odrive_controller_node,
        led_node,
        serial_estop_monitor_node,
    ])
