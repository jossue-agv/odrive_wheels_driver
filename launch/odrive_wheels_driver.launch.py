from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='agv',
            description='Robot namespace'),

        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel',
            description='cmd_vel input topic (use cmd_vel_safe when velocity smoother + collision monitor active)'),

        Node(
            package='odrive_wheels_driver',
            executable='odrive_wheels_driver_node',
            name='odrive_wheels_driver_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            output='screen',
        ),
    ])
