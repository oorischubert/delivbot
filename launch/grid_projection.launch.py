import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_delivbot = get_package_share_directory('delivbot')
    default_params = os.path.join(pkg_delivbot, 'config', 'octomap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    cloud_topic = LaunchConfiguration('cloud_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if available.'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Parameter YAML for octomap_server.'
    )
    declare_cloud = DeclareLaunchArgument(
        'cloud_topic', default_value='/map',
        description='Input 3D point cloud to project into a 2D occupancy grid.'
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', cloud_topic), ('/cloud_in', cloud_topic)],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_cloud,
        octomap,
    ])
