import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('trolleybot_bringup')
    pkg_description = get_package_share_directory('trolleybot_description')
    pkg_slam = get_package_share_directory('trolleybot_slam')

    # 1. State Publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'rsp.launch.py')]),
    )

    # 2. Base Node (Serial Communication)
    base_node = Node(
        package='trolleybot_base',
        executable='base_node',
        name='trolleybot_base_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0', # Adjust if Nano gets /dev/ttyUSB1
            'baudrate': 115200,
            'wheel_radius': 0.033,
            'wheel_separation': 0.16,
            'ticks_per_rev': 330.0
        }]
    )

    # 3. Lidar Node (Assuming standard sllidar_ros2 or rplidar_ros package structure)
    # The actual executable name may vary based on whether you install sllidar_ros2 or rplidar_ros.
    # We will assume rplidar_ros for standard ROS2 RPLidar A1M8 setup.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':'serial',
                     'serial_port': '/dev/ttyUSB1', # Adjust if Lidar gets /dev/ttyUSB0
                     'serial_baudrate': 115200,
                     'frame_id': 'laser_frame',
                     'inverted': False,
                     'angle_compensate': True}],
        output='screen'
    )

    # 4. SLAM Toolbox
    slam_params_file = os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          slam_params_file,
          {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        rsp_launch,
        base_node,
        TimerAction(period=2.0, actions=[lidar_node]), # Wait for 2s before starting lidar
        TimerAction(period=5.0, actions=[slam_node])  # Wait for 5s before starting SLAM
    ])
