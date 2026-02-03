
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')
    rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'map_merge_tb1_tb2.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='merge_map',
            description='Frame ID for the merged map'
        ),
        
        DeclareLaunchArgument(
            'robot_count', 
            default_value='3', 
            description='Number of robots'),
        
        Node(
            package='merge_map',
            executable='merge_map',
            name='merge_map',
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')},
                         {'robot_count': LaunchConfiguration('robot_count')},
                         {'use_sim_time': True}]
        ),
    ])
