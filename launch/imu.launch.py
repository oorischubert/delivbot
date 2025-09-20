import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_delivbot = get_package_share_directory('delivbot')
    default_params = os.path.join(pkg_delivbot, 'config', 'bno055.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    pkg = LaunchConfiguration('package')
    exe = LaunchConfiguration('executable')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace to apply to IMU node.'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time.'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='BNO055 parameter YAML file.'
    )
    declare_pkg = DeclareLaunchArgument(
        'package', default_value='bno055', description='BNO055 driver package name.'
    )
    declare_exe = DeclareLaunchArgument(
        'executable', default_value='bno055', description='BNO055 driver executable.'
    )

    # BNO055 node
    imu_node = Node(
        package=pkg,
        executable=exe,
        namespace=namespace,
        name='bno055',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # Provide flat aliases without namespace for downstream consumers
            ('bno055/imu', '/imu'),
            ('bno055/imu_raw', '/imu_raw'),
            ('bno055/temp', '/temperature'),
            ('bno055/mag', '/mag'),
            ('bno055/grav', '/gravity'),
            ('bno055/calib_status', '/calib_status'),
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params,
        declare_pkg,
        declare_exe,
        imu_node,
    ])

# package: https://github.com/flynneva/bno055?utm_source=chatgpt.com
# Install:
# git clone https://github.com/flynneva/bno055.git 