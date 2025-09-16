import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    parameters = [{
        'use_sim_time': use_sim_time.lower() in ('true', '1', 'yes')
    }]

    if params_file:
        parameters.append(os.path.expanduser(params_file))

    lidar_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        namespace=namespace,
        name=node_name,
        output='screen',
        parameters=parameters,
    )

    return [lidar_node]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the Hesai lidar driver.'
    )

    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='hesai_lidar',
        description='Node name for the Hesai lidar driver.'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional YAML parameters file for the Hesai driver.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set true if the lidar driver should use simulation time.'
    )

    return LaunchDescription([
        declare_namespace,
        declare_node_name,
        declare_params,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
