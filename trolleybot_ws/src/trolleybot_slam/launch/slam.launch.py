import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam = get_package_share_directory('trolleybot_slam')
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
        slam_node
    ])
