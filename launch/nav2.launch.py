import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_delivbot = get_package_share_directory('delivbot')

    default_nav2_params = os.path.join(pkg_delivbot, 'config', 'nav2_params.yaml')
    default_ekf_params = os.path.join(pkg_delivbot, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    lidar_odom_topic = LaunchConfiguration('lidar_odom_topic')
    filtered_odom_topic = LaunchConfiguration('filtered_odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the Nav2 parameter file to load.'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically bring up the Nav2 lifecycle nodes.'
    )

    declare_wheel_odom = DeclareLaunchArgument(
        'wheel_odom_topic',
        default_value='/odom',
        description='Odometry topic produced by the wheel encoders.'
    )

    declare_lidar_odom = DeclareLaunchArgument(
        'lidar_odom_topic',
        default_value='/lidar/odometry',
        description='Odometry topic computed from the lidar SLAM pipeline.'
    )

    declare_filtered_odom = DeclareLaunchArgument(
        'filtered_odom_topic',
        default_value='/odometry/filtered',
        description='Output topic name for fused odometry from robot_localization.'
    )

    declare_cmd_vel = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic to feed into the base (after twist_mux).'
    )

    nav2_param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=nav2_param_substitutions,
        convert_types=True
    )

    ekf_param_substitutions = {
        'use_sim_time': use_sim_time,
        'odom0': wheel_odom_topic,
        'odom1': lidar_odom_topic,
    }

    ekf_params = RewrittenYaml(
        source_file=default_ekf_params,
        root_key='ekf_localization_node',
        param_rewrites=ekf_param_substitutions,
        convert_types=True
    )

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', filtered_odom_topic)]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', cmd_vel_topic)]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    recoveries_server = Node(
        package='nav2_behaviors',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', cmd_vel_topic)]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_wheel_odom)
    ld.add_action(declare_lidar_odom)
    ld.add_action(declare_filtered_odom)
    ld.add_action(declare_cmd_vel)

    ld.add_action(ekf_localization)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(smoother_server)
    ld.add_action(recoveries_server)
    ld.add_action(bt_navigator)
    ld.add_action(velocity_smoother)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager)

    return ld
