import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Default LiDAR SLAM bring-up using lidarslam_ros2."""

    pkg_delivbot = get_package_share_directory('delivbot')
    default_params = os.path.join(pkg_delivbot, 'config', 'lidarslam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_imu = LaunchConfiguration('use_imu')
    use_odom = LaunchConfiguration('use_odom')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_qx = LaunchConfiguration('initial_pose_qx')
    initial_pose_qy = LaunchConfiguration('initial_pose_qy')
    initial_pose_qz = LaunchConfiguration('initial_pose_qz')
    initial_pose_qw = LaunchConfiguration('initial_pose_qw')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if available.'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Parameter file for lidarslam_ros2 front-end and back-end.'
    )
    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/lidar/points',
        description='Input point cloud topic.'
    )
    declare_imu = DeclareLaunchArgument(
        'imu_topic', default_value='/imu',
        description='Input IMU topic (optional, enable use_imu to fuse).'
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot base frame id.'
    )
    declare_map_frame = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Global map frame id.'
    )
    declare_use_imu = DeclareLaunchArgument(
        'use_imu', default_value='false',
        description='Set false to disable IMU fusion in scan matcher.'
    )
    declare_use_odom = DeclareLaunchArgument(
        'use_odom', default_value='true',
        description='Set true to use wheel odometry as an initial guess when available.'
    )
    declare_initial_pose = DeclareLaunchArgument(
        'initial_pose_topic', default_value='initial_pose',
        description='Topic delivering the initial pose PoseStamped message.'
    )
    declare_auto_initial_pose = DeclareLaunchArgument(
        'auto_initial_pose', default_value='true',
        description='When true, publish an initial pose automatically after launch.'
    )
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0', description='Initial pose X coordinate.'
    )
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0', description='Initial pose Y coordinate.'
    )
    declare_initial_pose_z = DeclareLaunchArgument(
        'initial_pose_z', default_value='0.0', description='Initial pose Z coordinate.'
    )
    declare_initial_pose_qx = DeclareLaunchArgument(
        'initial_pose_qx', default_value='0.0', description='Initial pose quaternion x.'
    )
    declare_initial_pose_qy = DeclareLaunchArgument(
        'initial_pose_qy', default_value='0.0', description='Initial pose quaternion y.'
    )
    declare_initial_pose_qz = DeclareLaunchArgument(
        'initial_pose_qz', default_value='0.0', description='Initial pose quaternion z.'
    )
    declare_initial_pose_qw = DeclareLaunchArgument(
        'initial_pose_qw', default_value='1.0', description='Initial pose quaternion w.'
    )
    declare_initial_pose_delay = DeclareLaunchArgument(
        'initial_pose_delay', default_value='0.5', description='Seconds to wait before publishing initial pose.'
    )

    # Apply frame and IMU rewrites to the parameter file
    scan_param_rewrites = {
        'global_frame_id': map_frame,
        'robot_frame_id': base_frame,
        'use_imu': use_imu,
        'use_odom': use_odom,
    }

    scan_params = RewrittenYaml(
        source_file=params_file,
        root_key='scan_matcher',
        param_rewrites=scan_param_rewrites,
        convert_types=True,
    )

    graph_params = RewrittenYaml(
        source_file=params_file,
        root_key='graph_based_slam',
        param_rewrites={},
        convert_types=True,
    )

    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scan_matcher',
        output='screen',
        parameters=[scan_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('input_cloud', pointcloud_topic),
            ('imu', imu_topic),
            ('initial_pose', initial_pose_topic),
        ]
    )

    backend_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        parameters=[graph_params, {'use_sim_time': use_sim_time}],
    )

    def make_initial_pose_action(context, *_, **__):
        auto = auto_initial_pose.perform(context).lower()
        if auto not in ('true', '1', 'yes'):
            return []

        topic = initial_pose_topic.perform(context)
        frame = map_frame.perform(context)
        x = initial_pose_x.perform(context)
        y = initial_pose_y.perform(context)
        z = initial_pose_z.perform(context)
        qx = initial_pose_qx.perform(context)
        qy = initial_pose_qy.perform(context)
        qz = initial_pose_qz.perform(context)
        qw = initial_pose_qw.perform(context)
        delay = float(initial_pose_delay.perform(context))

        params = {
            'topic': topic,
            'frame_id': frame,
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'qx': float(qx),
            'qy': float(qy),
            'qz': float(qz),
            'qw': float(qw),
            'delay': float(delay),
        }

        return [
            Node(
                package='delivbot',
                executable='initial_pose_publisher',
                name='auto_initial_pose',
                output='screen',
                parameters=[params],
            )
        ]

    initial_pose_action = OpaqueFunction(function=make_initial_pose_action)

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_pointcloud,
        declare_imu,
        declare_base_frame,
        declare_map_frame,
        declare_use_imu,
        declare_use_odom,
        declare_initial_pose,
        declare_auto_initial_pose,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_z,
        declare_initial_pose_qx,
        declare_initial_pose_qy,
        declare_initial_pose_qz,
        declare_initial_pose_qw,
        declare_initial_pose_delay,
        scanmatcher_node,
        backend_node,
        #initial_pose_action,
    ])

# Package:
# https://github.com/rsasaki0109/lidarslam_ros2/tree/develop
# Install:
# git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
# cd ..
# rosdep install --from-paths src --ignore-src -r -y

# ros2 topic pub --once /initial_pose geometry_msgs/PoseStamped \ '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

# View map:
# sudo apt install -y pcl-tools
# pcl_viewer map.pcd
