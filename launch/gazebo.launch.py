import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('delivbot')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'delivbot.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    package_name = 'delivbot'

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # ROS 2 control controller manager using Ignition plugin
    control_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros2_control.yaml')

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, control_config],
        output='both'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Define the twist_mux node
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    world_path = os.path.join(pkg_ros_gz_rbot, 'worlds')

    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', world_path
    )

    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            'gz_args': ["-r -v4 ", world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot in Gazebo   
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "delivbot",
            "-allow_renaming", "false",
            "-z", "0.32",
            "-x", "0.0",
            "-y", "-0.25",
            "-Y", "0.0"
        ],            
        output='screen',
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
        }],
        output='screen'
      )  
    
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/realsense/image"]
    )    

    lidar_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_1', 'delivbot/base_link/lidar_1'],
        output='screen'
    )

    realsense_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'realsense_1', 'delivbot/base_link/realsense_1'],
        output='screen'
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            world_arg,
            set_ign_resource_path, # is this neccesary?
            gazebo,
            spawn,
            start_gazebo_ros_bridge_cmd,
            lidar_frame_alias,
            realsense_frame_alias,
            ros_gz_image_bridge,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            joystick,
            twist_mux,
        ]
    )
    
# export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/
# ps aux | grep gzserver   <- reset background segmentation fault

#warehouse world launch:
# ros2 launch delivbot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
#house world launch:
# ros2 launch delivbot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-small-house-world/worlds/small_house.world
#bookstore world launch:
# ros2 launch delivbot gazebo.launch.py world:=$HOME/ros2_ws/src/aws-robomaker-bookstore-world/worlds/bookstore.world

#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git 
#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git
#git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
