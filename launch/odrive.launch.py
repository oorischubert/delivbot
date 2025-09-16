import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    parameters = [{
        'cmd_vel_topic': cmd_vel_topic,
        'use_sim_time': use_sim_time.lower() in ('true', '1', 'yes')
    }]

    if params_file:
        parameters.append(os.path.expanduser(params_file))

    remappings = []
    if cmd_vel_topic:
        remappings.append(('/cmd_vel', cmd_vel_topic))

    odrive_node = Node(
        package='odrive_hoverboard',
        executable='driver',
        namespace=namespace,
        name=node_name,
        output='screen',
        parameters=parameters,
        remappings=remappings,
    )

    return [odrive_node]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the ODrive hoverboard driver.'
    )

    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='odrive_hoverboard',
        description='Node name for the ODrive hoverboard driver.'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional YAML parameters file for the ODrive driver.'
    )

    declare_cmd_vel = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Twist topic that provides velocity commands to the driver.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set true if the driver should use simulation time (normally false for hardware).'
    )

    return LaunchDescription([
        declare_namespace,
        declare_node_name,
        declare_params,
        declare_cmd_vel,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
