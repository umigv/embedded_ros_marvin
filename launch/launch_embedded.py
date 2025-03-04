import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    # Declare launch arguments
    use_LED_arg = launch.substitutions.LaunchConfiguration('use_LED')
    use_enc_tf_arg = launch.substitutions.LaunchConfiguration('use_enc_tf')

    # Define the LED node (only launched if use_LED is true)
    led_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='LED_subscriber',
        output='screen',
        condition=launch.conditions.IfCondition(use_LED_arg)
    )

    # These nodes always launch
    dual_odrive_controller_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='dual_odrive_controller',
        output='screen'
    )

    enc_odom_publisher_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='enc_odom_publisher',
        output='screen',
        parameters=[{'use_enc_tf': use_enc_tf_arg}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_LED', default_value='false', description="Enable LED node"),
        launch.actions.DeclareLaunchArgument('use_enc_tf', default_value='false', description="Enable TF publishing using encoder odometry"),
        dual_odrive_controller_node,
        enc_odom_publisher_node,
        led_node
    ])
