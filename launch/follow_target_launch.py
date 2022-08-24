from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('follow_target'),
        'config', 'follow_target.yaml'
    ])
    default_controller = PathJoinSubstitution([
        FindPackageShare('follow_target'),
        'config', 'default_controller.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        Node(
            package='follow_target',
            executable='follow_target_node',
            name='follow_target',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": True},
                config,
                default_controller
                ],
            output='screen',
            emulate_tty=True
        )
    ])