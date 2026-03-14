import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    # Declare launch arguments
    use_LED_arg = launch.substitutions.LaunchConfiguration('use_LED')

    led_node = launch_ros.actions.Node(
        package='embedded_ros_marvin', 
        executable='LED_subscriber',
        output='screen',
        condition=launch.conditions.IfCondition(use_LED_arg)
    )

    dual_odrive_controller_node = launch_ros.actions.Node(
        package='embedded_ros_marvin', 
        executable='dual_odrive_controller',
        output='screen',
        remappings=[('enc_vel', 'enc_vel/raw')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_LED', default_value='true', description="Enable LED node"),
        dual_odrive_controller_node,
        led_node
    ])
