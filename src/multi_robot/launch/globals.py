# (c) robotics.snowcron.com
# Use: MIT license

import os
from ament_index_python.packages import get_package_share_directory

global def_ros_shared_path
def_ros_shared_path = '/opt/ros/humble/share/'

def_package_name = 'multi_robot'

global def_bringup_dir
def_bringup_dir = os.path.expanduser("~/ros2_multirobot_ws/src/" + def_package_name)

global def_launch_dir
def_launch_dir = os.path.join(def_bringup_dir, 'launch')

global def_maps_path
def_maps_path = os.path.join(def_bringup_dir, '../maps', 'map.yaml')

global def_keepout_mask_path
def_keepout_mask_path = os.path.join(def_bringup_dir, '../maps', 'keepout_mask.yaml')

global def_nav2_bt_navigator_path
def_nav2_bt_navigator_path = os.path.join(def_ros_shared_path, 'nav2_bt_navigator',
    'behavior_trees', 
    # Galactic: 
    # 'navigate_to_pose_w_replanning_and_recovery.xml'
    # Foxy:
    "navigate_w_replanning_and_recovery.xml"
    )

global def_nav2_params_path
def_nav2_params_path = os.path.join(def_bringup_dir, 'config', 'nav2_params.yaml')

global def_nav2_params_multi_path
def_nav2_params_path_multi = os.path.join(def_bringup_dir, 'config', 'nav2_params_multi.yaml')

# Map fully qualified names to relative ones so the node's namespace can be prepended.
# In case of the transforms (tf), currently, there doesn't seem to be a better alternative
# https://github.com/ros/geometry2/issues/32
# https://github.com/ros/robot_state_publisher/pull/30
# TODO(orduno) Substitute with `PushNodeRemapping`
#              https://github.com/ros2/launch_ros/issues/56
global def_remappings
def_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

global def_rviz_path
def_rviz_path = os.path.join(def_bringup_dir, 'rviz', 'robot.rviz')

global def_urdf
def_urdf = os.path.join(def_bringup_dir,'description','robot.urdf.xacro')

global def_world_path
def_world_path = os.path.join(def_bringup_dir, '../worlds', 'shapes.sdf')

# global namespaced_params_var
# namespaced_params_var = 'params_file'

global namespaced_params
global namespaced_default_bt_xml_filename

# ---

# global getModifiedNav2Params
# def getModifiedNav2Params(strNav2Path, arrReplacements):
#     strConfig = None
    
#     with open(strNav2Path, 'r') as file:
#         strConfig = file.read()

#         for rec in arrReplacements:
#             strConfig = strConfig.replace(rec[0], rec[1])

#     return strConfig

# ---
