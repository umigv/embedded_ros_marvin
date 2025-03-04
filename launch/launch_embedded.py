import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    # Declare launch arguments
    use_LED_arg = launch.substitutions.LaunchConfiguration('use_LED')
    use_enc_odom_arg = launch.substitutions.LaunchConfiguration('use_enc_odom')

    led_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='LED_subscriber',
        output='screen',
        condition=launch.conditions.IfCondition(use_LED_arg)
    )

    dual_odrive_controller_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='dual_odrive_controller',
        output='screen'
    )

    enc_odom_publisher_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='enc_odom_publisher',
        output='screen',
        condition=launch.conditions.IfCondition(use_enc_odom_arg)
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_LED', default_value='true', description="Enable LED node"),
        launch.actions.DeclareLaunchArgument('use_enc_odom', default_value='false', description="Enable encoder odometry and TF"),
        dual_odrive_controller_node,
        enc_odom_publisher_node,
        led_node
    ])
