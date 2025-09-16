import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def robot_state_publisher_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    model_path = LaunchConfiguration('model').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    robot_description_config = xacro.process_file(os.path.expanduser(model_path))
    robot_description = robot_description_config.toxml()

    parameters = [{
        'robot_description': robot_description,
        'use_sim_time': use_sim_time.lower() in ('true', '1', 'yes')
    }]

    node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        output='both',
        parameters=parameters,
    )

    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('delivbot')
    default_model = os.path.join(pkg_share, 'urdf', 'delivbot.xacro')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    joystick_x_mode_params = os.path.join(pkg_share, 'config', 'joystick_x_mode.yaml')

    declare_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace to apply to all robot nodes.'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the robot description XACRO file.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )

    declare_launch_lidar = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Launch the physical lidar driver when true.'
    )

    declare_launch_realsense = DeclareLaunchArgument(
        'launch_realsense',
        default_value='true',
        description='Launch the RealSense camera driver when true.'
    )

    declare_launch_odrive = DeclareLaunchArgument(
        'launch_odrive',
        default_value='true',
        description='Launch the ODrive hoverboard driver when true.'
    )

    declare_launch_joystick = DeclareLaunchArgument(
        'launch_joystick',
        default_value='true',
        description='Launch joystick teleop for manual control when true.'
    )

    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/f710-joystick',
        description='Joystick device path (e.g., /dev/input/f710-joystick). If empty, joystick.yaml device_id is used.'
    )

    declare_joystick_params = DeclareLaunchArgument(
        'joystick_params_file',
        default_value=joystick_x_mode_params,
        description='YAML parameters file for joystick (defaults to X-mode config).'
    )

    declare_lidar_params = DeclareLaunchArgument(
        'lidar_params_file',
        default_value='',
        description='Optional YAML parameters file for the lidar driver.'
    )

    declare_realsense_params = DeclareLaunchArgument(
        'realsense_params_file',
        default_value='',
        description='Optional YAML parameters file for the RealSense driver.'
    )

    declare_odrive_params = DeclareLaunchArgument(
        'odrive_params_file',
        default_value='',
        description='Optional YAML parameters file for the ODrive driver.'
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Twist topic that provides velocity commands to the ODrive driver.'
    )

    launch_items = [
        declare_namespace,
        declare_model,
        declare_use_sim_time,
        declare_launch_lidar,
        declare_launch_realsense,
        declare_launch_odrive,
        declare_launch_joystick,
        declare_joy_dev,
        declare_joystick_params,
        declare_lidar_params,
        declare_realsense_params,
        declare_odrive_params,
        declare_cmd_vel_topic,
        OpaqueFunction(function=robot_state_publisher_setup),
    ]

    namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=namespace,
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', LaunchConfiguration('cmd_vel_topic'))],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'lidar.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_lidar')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('lidar_params_file'),
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'realsense.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_realsense')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('realsense_params_file'),
        }.items(),
    )

    odrive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'odrive.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_odrive')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('odrive_params_file'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }.items(),
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'joystick.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_joystick')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'joy_dev': LaunchConfiguration('joy_dev'),
            'params_file': LaunchConfiguration('joystick_params_file'),
        }.items(),
    )

    lidar_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=["0.63", "0.175", "0.564", "0", "0", "0", "base_link", "hesai_lidar"],
    )
    camera_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=["0.675", "0.175", "0.50975", "0", "0", "0", "base_link", "camera_link"],
    )

    launch_items.extend([
        twist_mux_node,
        lidar_launch,
        realsense_launch,
        odrive_launch,
        joystick_launch,
        lidar_static_tf_node,
        camera_static_tf_node,
    ])

    return LaunchDescription(launch_items)
