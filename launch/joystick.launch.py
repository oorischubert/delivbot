from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')
    params_file = LaunchConfiguration('params_file')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'dev': joy_dev}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='',
            description='Path to joystick device (leave empty to use device_id from YAML)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('delivbot'),
                'config',
                'joystick_x_mode.yaml'
            ),
            description='Joystick parameters YAML file (defaults to X-mode mapping)'
        ),
        joy_node,
        teleop_node,
    ])
