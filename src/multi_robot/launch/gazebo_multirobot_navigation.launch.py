#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        
        #my_world
        {'name': 'tb1', 'x_pose': '5.0', 'y_pose': '4.0', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 },
        {'name': 'tb2', 'x_pose': '-4.5', 'y_pose': '4.0', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
        {'name': 'tb3', 'x_pose': '6.0', 'y_pose': '-1.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
        {'name': 'tb4', 'x_pose': '-4.0', 'y_pose': '-4.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
        # {'name': 'tb5', 'x_pose': '0.0', 'y_pose': '-0.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
        # {'name': 'tb6', 'x_pose': '0.0', 'y_pose': '0.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
        
        # Add more robots here
    ]
    
    print(f"Number of robots: {len(robots)}\n\n")
    
    # Declare robot_count argument
    robot_count = LaunchConfiguration('robot_count', default=str(len(robots)))
    declare_robot_count = DeclareLaunchArgument(
        name='robot_count',
        default_value=robot_count,
        description='Number of robots to spawn'
    )
    
    ld.add_action(declare_robot_count)

    MY_ROBOT = 'turtlebot3_waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulation time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable RViz launch'
    )
    multi_robot = get_package_share_directory('multi_robot')
    
    package_dir = get_package_share_directory('multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    urdf = os.path.join(
        multi_robot, 'urdf', MY_ROBOT + '.urdf'
    )
    model = os.path.join(
        multi_robot,'models', MY_ROBOT, 'model.sdf'
    )

    world = os.path.join(multi_robot,'worlds', 'my_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )
    
    map_file_path = os.path.join(
    multi_robot,
    '../../../../src/saved_map/multi_robot_autonomous_mapping_of_'+str(len(robots))+'_robots.yaml')

    # multi_robot_autonomous_mapping_of_4_robots.pgm
    print(f"Map file path: {map_file_path}\n\n")


    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:
        namespace =  [ '/' + robot['name'] ]
        
        params_file = os.path.join(get_package_share_directory('multi_robot'), 'params', f"nav2_params_{robot['name']}_0.yaml")
        print(f"Namespace: {namespace}, PARAM Config: {params_file}\n\n")
        
                    # Map server and lifecycle manager
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=namespace,
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}],
            remappings=remappings,
        )

        map_server_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace,
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'autostart': True}, 
                        {'node_names': ['map_server']}]
        )
        
        # Create state publisher node for that instance
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )
        
        # Joint State Publisher node
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            name='joint_state_publisher',
            parameters=[{'use_sim_time':use_sim_time}],
            remappings=remappings,
        )

        # Create spawn call
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', model,
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0', '-R', '0.0', '-P', '0.0',
            ],
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('multi_robot'), 'launch', 'nav2_bringup', 'bringup_launch.py')),
                launch_arguments={
                    'slam': 'False',
                    'namespace': namespace,
                    'use_namespace': 'True',
                    'map': map_file_path,
                    'map_server': 'False',
                    'params_file': params_file,
                    'default_bt_xml_filename': os.path.join(
                        get_package_share_directory('nav2_bt_navigator'),
                        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                    'autostart': 'True',
                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items())
    
        node_tf_map_to_odom = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0', '0', '0', '0',
            robot['name'] + '/map', robot['name'] + '/odom'],
            parameters=[{'use_sim_time': True}],
            output='screen')

        if last_action is None:
            # Call add_action directly for the first robot
            ld.add_action(map_server)
            ld.add_action(map_server_lifecycle)
            ld.add_action(robot_state_publisher)
            ld.add_action(spawn_robot)
            ld.add_action(bringup_cmd)
            # ld.add_action(robot_localization_node)
            ld.add_action(joint_state_publisher_node)
            ld.add_action(node_tf_map_to_odom)
        else:
            # Ensure next robot creation happens only after the previous one is completed
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_robot, 
                             robot_state_publisher, 
                             joint_state_publisher_node, 
                             node_tf_map_to_odom, 
                             bringup_cmd,
                             map_server,
                             map_server_lifecycle
                             ],
                )
            )
            ld.add_action(spawn_robot_event)

        # Save last instance for the next RegisterEventHandler
        last_action = spawn_robot
    

    for robot in robots:
        namespace =  [ '/' + robot['name']]

        rviz_config_file_ = os.path.join(get_package_share_directory('multi_robot'), 'rviz', f"multi_robot_maping_{robot['name']}.rviz")

        print(f"Namespace: {namespace}, RViz Config: {rviz_config_file_}\n\n")
    
        
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file_, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3 = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # Ensure initial pose and drive nodes start only after robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[rviz_cmd, drive_turtlebot3],
            )
        )
        
        ld.add_action(post_spawn_event)

    return ld
