import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    parameters = [{
        'use_sim_time': use_sim_time.lower() in ('true', '1', 'yes')
    }]

    if params_file:
        parameters.append(os.path.expanduser(params_file))

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=namespace,
        name=camera_name,
        output='screen',
        parameters=parameters,
        remappings=[
            ('/realsense_camera/color/image_raw', '/realsense/image'),
            ('/realsense_camera/color/camera_info', '/realsense/camera_info'),
            ('/realsense_camera/depth/image_rect_raw', '/realsense/depth_image'),
            ('/realsense_camera/depth/camera_info', '/realsense/depth_camera_info'),
            ('/realsense_camera/depth/color/points', '/realsense/points'),
        ],
    )

    return [camera_node]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to apply to the RealSense driver.'
    )

    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='realsense_camera',
        description='Node name for the RealSense driver.'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional YAML parameters file for the RealSense node.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set true if the RealSense driver should use simulation time.'
    )

    return LaunchDescription([
        declare_namespace,
        declare_camera_name,
        declare_params,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
