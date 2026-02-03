##Give the path of the partial maps that need to be merged and the robot count need to match the .yaml file in the dir

##ros2 launch merge_map offline_merge_map_launch.py robot_count:=2 maps_directory:="/home/tsm02/debug_robot/maps/robot/" frame_id:="map"


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    maps_directory = LaunchConfiguration('maps_directory').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)

    # Validate that the maps directory exists
    if not os.path.isdir(maps_directory):
        raise ValueError(f"Maps directory '{maps_directory}' does not exist!")

    # Get all YAML files in the maps directory
    map_files = [f for f in os.listdir(maps_directory) if f.endswith('.yaml')]

    # Ensure we have enough map files for each robot
    if len(map_files) != robot_count:
        raise ValueError(f"Expected {robot_count} map files, but found {len(map_files)}.")

    # Log the robot count and map files
    file_paths = ', '.join([os.path.join(maps_directory, f) for f in map_files])
    launch_actions = [LogInfo(msg=f"Launching merge_map with {robot_count} robots and map files: {file_paths}")]

    # Start map_server nodes with dynamically generated parameters
    for i in range(robot_count):
        yaml_filename = os.path.join(maps_directory, map_files[i])

        # Dynamically generate parameters for each map server
        map_server_params = {
            'frame_id': frame_id,  # common frame_id for all robots
            'topic_name': f'tb{i+1}/map',  # Dynamically assign topic based on robot number
            'use_sim_time': False,
            'yaml_filename': yaml_filename,
        }

        # Start map_server node with generated parameters
        node = Node(
            package='nav2_map_server',
            executable='map_server',
            name=f'map_server_{i+1}',
            # namespace=f'tb{i+1}',
            output='screen',
            parameters=[map_server_params]
        )

        launch_actions.append(node)

        # Transition the map server through lifecycle states using ExecuteProcess
        configure = TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg=f"Configuring map_server_{i+1}..."),
                ExecuteProcess(
                    # cmd=["ros2", "lifecycle", "set", f"/tb{i+1}/map_server_{i+1}", "configure"],
                    cmd=["ros2", "lifecycle", "set", f"/map_server_{i+1}", "configure"],
                    output="screen"
                ),
            ],
        )
        activate = TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg=f"Activating map_server_{i+1}..."),
                ExecuteProcess(
                    # cmd=["ros2", "lifecycle", "set", f"/tb{i+1}/map_server_{i+1}", "activate"],
                    cmd=["ros2", "lifecycle", "set", f"/map_server_{i+1}", "activate"],
                    output="screen"
                ),
            ],
        )
        launch_actions.append(configure)
        launch_actions.append(activate)

    # Merge map node
    merge_map_node = Node(
        package='merge_map',
        executable='offline_merge_map',
        name='offline_merge_map',
        output='screen',
        parameters=[{'frame_id': frame_id}, {'robot_count': robot_count}],
    )
    launch_actions.append(merge_map_node)
    
    # RViz node
    rviz_file = os.path.join(get_package_share_directory('merge_map'), 'config', 'offline_merge_map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}]
    )

    launch_actions.append(rviz_node)

    return launch_actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='2', description='Number of robots'),
        DeclareLaunchArgument('maps_directory', default_value='', description='Directory containing map YAML files'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Frame ID for the merged map'),
        OpaqueFunction(function=launch_setup),
    ])