# Technical Reference Manual

## Multi-Robot Autonomous Mapping (ROS 2 Humble)

This document is the **official Technical Reference Manual** for the multi-robot autonomous mapping system. It is intended for developers, researchers, and maintainers who need detailed understanding of launch files, parameters, internal node behavior, and system architecture.


---

# 1. Core Launch File

## gazebo_multirobot_mapping_with_nav2.launch.py

**File Location:**

```
ros2_humble/multi_robot_autonomous/src/multi_robot/launch/gazebo_multirobot_mapping_with_nav2.launch.py
```

This launch file is responsible for:

* Spawning multiple robots in Gazebo
* Namespacing each robot
* Launching Nav2 for each robot
* Launching multi-robot SLAM Toolbox
* Launching merge_map
* Launching multi_robot_exploration control node
* Managing startup sequencing and event handling

---

# 2. Robot Spawn Configuration

## robots Array (LINE ~18)

Defines which robots are spawned and their initial pose in Gazebo.

```python
robots = [
  {'name': 'tb1', 'x_pose': '5.0',  'y_pose': '4.0',  'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
  {'name': 'tb2', 'x_pose': '-4.5','y_pose': '4.0',  'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
  {'name': 'tb3', 'x_pose': '6.0',  'y_pose': '-1.5','z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
  {'name': 'tb4', 'x_pose': '-4.0','y_pose': '-4.5','z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
  # {'name': 'tb5', 'x_pose': '0.0', 'y_pose': '-0.5','z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
  # {'name': 'tb6', 'x_pose': '0.0', 'y_pose': '0.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
]
```

### Notes

* Comment or uncomment robot entries to enable/disable robots
* Each robot must have a unique name (tb1, tb2, ...)
* Pose values define Gazebo spawn location

---

# 3. robot_count Launch Argument

**LINE ~36**

```python
robot_count = LaunchConfiguration('robot_count', default='2')
```

### Purpose

Controls how many robots are:

* Passed to merge_map
* Passed to multi_robot_exploration
* Used for internal logic

### Critical Requirement

👉 `robot_count` MUST match the number of active robots in the `robots` array.

---

# 4. merge_map Launch

**LINE ~45**

```python
merge_map_launch = ExecuteProcess(
  cmd=['ros2', 'launch', 'merge_map', 'merge_map_launch.py', ['robot_count:=', robot_count]],
  output='screen'
)
```

### Function

* Launches map merging process
* Subscribes to /tbX/map
* Publishes /merge_map

---

# 5. multi_robot_exploration Control Node

**LINE ~52**

```python
control_node = ExecuteProcess(
  cmd=['ros2', 'run', 'multi_robot_exploration', 'control', '--ros-args', '-p', ['robot_count:=', robot_count]],
  output='screen'
)
```

### Function

* Frontier detection
* Goal assignment
* Autonomous exploration
* Continuous merged map saving

### Map Save Path

```python
saved_map/multi_robot_autonomous_mapping_of_<N>_robots
```

---

# 6. Robot Model Selection

**LINE ~59**

```python
MY_ROBOT = 'turtlebot3_waffle'
```

### Supported Models

* turtlebot3_waffle
* turtlebot3_burger

---

# 7. Gazebo World Selection

**LINE ~87**

```python
world = os.path.join(multi_robot, 'worlds', 'my_world.world')
```

### Notes

* Change world file to use custom Gazebo environments
* Worlds must be located in:

```
src/multi_robot/worlds/
```

---

# 8. Nav2 Parameters Per Robot

**LINE ~116**

```python
params_file = os.path.join(
  get_package_share_directory('multi_robot'),
  'params',
  f"nav2_params_{robot['name']}_0.yaml"
)
```

### Purpose

Each robot has an independent Nav2 configuration:

* Initial AMCL pose
* Localization tuning
* Controller parameters
* Planner parameters

### Files

```
params/nav2_params_tb1_0.yaml
params/nav2_params_tb2_0.yaml
...
```

---

# 9. Multi-Robot SLAM Toolbox

**LINE ~174**

```python
online_async_multirobot_launch.py
```

### Notes

* Uses modified slam_toolbox for multi-robot SLAM
* Original upstream slam_toolbox is listed in README dependencies

---

# 10. Launch File Architecture

## Imports and Setup

* ament_index_python
* launch
* launch_ros.actions
* launch.event_handlers
* launch.conditions

These provide:

* Package path resolution
* Node execution
* Conditional launch logic
* Event-based sequencing

---

# 11. generate_launch_description() Flow

## Robot Configuration

* robots list defines all robot entities

## Launch Arguments

* robot_count
* use_sim_time
* enable_drive
* enable_rviz

## Gazebo Setup

* gzserver_cmd (Gazebo server)
* gzclient_cmd (Gazebo client)

## Per-Robot Nodes

For each robot:

* robot_state_publisher
* joint_state_publisher_node
* spawn_robot
* bringup_cmd (Nav2)
* slam_toolbox_node

---

# 12. Event Handling and Sequential Startup

Uses RegisterEventHandler and OnProcessExit to:

* Spawn robots sequentially
* Ensure stable startup
* Delay RViz and drive nodes

```python
RegisterEventHandler(
  OnProcessExit(
    target_action=last_action,
    on_exit=[rviz_cmd, drive_turtlebot3]
  )
)
```

---

# 13. Namespacing and TF Remapping

## Namespacing

```python
namespace = ['/' + robot['name']]
```

Each robot runs in its own ROS namespace:

* /tb1
* /tb2
* /tb3
* ...

## TF Remapping

```python
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
```

Prevents TF collisions in multi-robot systems.

---

# 14. Control Node — multi_robot_exploration

## Overview

Implements frontier-based autonomous exploration for multiple robots.

### Features

* Dynamic multi-robot handling
* Frontier detection and grouping
* Goal assignment
* NavigateToPose action usage
* Exploration completion detection
* Automatic merged map saving

### Core Components

#### Frontier Detection

* frontierB(matrix)
* assign_groups(matrix)
* fGroups(groups)

#### Target Selection

* findClosestGroup()
* exploration()

#### HeadquartersControl Node

Subscribers:

* /merge_map (OccupancyGrid)
* /tbX/odom (Odometry)
* /tbX/cmd_vel (Twist)

Action Clients:

* /tbX/navigate_to_pose

Key Methods:

* map_callback()
* robot_odom_callback()
* send_goal()
* goal_result_callback()
* check_exploration_completion()
* save_map()

---

# 15. Merge Map Node — merge_map

## Overview

Merges occupancy grids from multiple robots into a single global map.

### Parameters

* frame_id (default: merge_map)
* robot_count (default: 3)

### Subscribed Topics

* /tb1/map
* /tb2/map
* ...

### Published Topics

* /merge_map

### Core Functions

* merge_maps()
* map_callback()
* try_merge_and_publish()

---

# 16. LaunchDescription Actions Summary

Key actions added to LaunchDescription:

```python
ld.add_action(declare_use_sim_time)
ld.add_action(declare_robot_count)
ld.add_action(declare_enable_drive)
ld.add_action(declare_enable_rviz)
ld.add_action(gzserver_cmd)
ld.add_action(gzclient_cmd)
ld.add_action(map_server)
ld.add_action(map_server_lifecycle)
ld.add_action(robot_state_publisher)
ld.add_action(spawn_robot)
ld.add_action(bringup_cmd)
ld.add_action(joint_state_publisher_node)
ld.add_action(node_tf_map_to_odom)
ld.add_action(slam_toolbox_node)
```

---

# 17. Offline Map Merging

## Additional feature - Offline Map Merging

* Merge multiple partially mapped maps into a single map 

Specify the directory containing the partial map YAML files. The number of YAML files in the directory must match the robot_count parameter.

```bash
ros2 launch merge_map offline_merge_map_launch.py robot_count:=2 maps_directory:="~/debug_robot/maps/" frame_id:="map"
```

### Overview

The offline map merging feature allows combining individual robot maps saved as YAML files into a single merged map without running the full simulation.

### Launch File Details

**File Location:**

```
ros2_humble/multi_robot_autonomous/src/merge_map/launch/offline_merge_map_launch.py
```

### Parameters

* `robot_count` (default: '2') - Number of robots/maps to merge
* `maps_directory` (required) - Directory containing the YAML map files
* `frame_id` (default: 'map') - Frame ID for the merged map

### Process

1. Validates the maps directory exists
2. Counts YAML files and ensures they match `robot_count`
3. Launches map_server nodes for each YAML file on topics `/tb1/map`, `/tb2/map`, etc.
4. Configures and activates each map_server via lifecycle
5. Launches the `offline_merge_map` node to merge the maps
6. Opens RViz with a configuration to visualize the merged map

### Requirements

* YAML files must be named appropriately (e.g., map1.yaml, map2.yaml)
* Each YAML file should correspond to a robot's partial map
* Directory path can use `~` for home directory

---

# 18. Multi-Robot Navigation with Pre-built Map

## Overview

The multi-robot navigation launch file allows testing navigation using a pre-built map from the multi-robot mapping process.

### Launch File Details

**File Location:**

```
ros2_humble/multi_robot_autonomous/src/multi_robot/launch/gazebo_multirobot_navigation.launch.py
```

### Launch Command

```bash
ros2 launch multi_robot gazebo_multirobot_navigation.launch.py robot_count:=4
```

### Parameters

* `robot_count` (default: '4') - Number of robots to spawn (dynamically set to len(robots))
* `use_sim_time` (default: 'true') - Use simulation time
* `enable_drive` (default: 'false') - Enable robot drive node
* `enable_rviz` (default: 'true') - Enable RViz launch

### Process

1. Prints the number of robots defined in the `robots` array
2. Constructs the map file path from the saved merged map
3. Launches Gazebo server and client
4. Spawns each robot sequentially with Nav2 stack
5. Loads the merged map for localization
6. Launches RViz for each robot with individual configurations
7. Allows goal assignment through RViz 2D Nav Goal tool

### Map File Path Configuration

The map file path is constructed in the launch file:

**LINE ~86**

```python
map_file_path = os.path.join(
    multi_robot,
    '../../../../src/saved_map/multi_robot_autonomous_mapping_of_'+str(robot_count)+'_robots')
```

**To change the map file path:**

1. Open `gazebo_multirobot_navigation.launch.py`
2. Modify the `map_file_path` variable to point to your desired map YAML file
3. Ensure the path is absolute or relative to the package share directory
4. The map should be a merged map from the multi-robot mapping process

### Robot Configuration

The `robots` array defines spawn positions:

```python
robots = [
    {'name': 'tb1', 'x_pose': '5.0', 'y_pose': '4.0', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
    {'name': 'tb2', 'x_pose': '-4.5', 'y_pose': '4.0', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
    {'name': 'tb3', 'x_pose': '6.0', 'y_pose': '-1.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
    {'name': 'tb4', 'x_pose': '-4.0', 'y_pose': '-4.5', 'z_pose': 0.01, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00},
]
```

### Notes

* Each robot runs in its own namespace (/tb1, /tb2, etc.)
* RViz configurations are loaded per robot for individual control
* Use the 2D Nav Goal tool in RViz to assign navigation goals
* The map is loaded for localization, not SLAM (slam: 'False')


