# Requirements:
#   Install Turtlebot3 packages
#   Note that we can edit turtlebot3_gazebo/models/turtlebot_waffle/model.sdf 
#     to increase min scan range from 0.12 to 0.2 to avoid having scans 
#     hitting the robot itself
#
#  original file is from  colcon_ws/src/rtabmap_ros/launch/ros2/turtlebot3_scan.launch.py
#
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_gazebo
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# Example:
#  1. Gazebo
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
#
#   Gass Station
#     ~/colcon_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world_nav_nishi2.launch.py
#    $ ros2 launch turtlebot3_gazebo turtlebot3_world_nav_nishi2.launch.py
#
#   2. rtabmap_ros with scan
#   SLAM:
#   $ ros2 launch rtabmap_ros_my turtlebot3_scan.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint subscribe_scan:=true depth:=false approx_sync:=true odom_topic:=/odom args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Reg/Force3DoF true --Grid/RangeMin 0.2" use_sim_time:=true
#
#   2.1 Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard
#
#   3. navigation2
#       default local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/scan/nav2_params.yaml
#       teb_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/scan/teb_nav2_params.yaml
#       rpp_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/scan/rpp_nav2_params.yaml
#
#   $ ros2 launch nav2_bringup rviz_launch.py
#
#   4. C++ Program controll
#   #$ ros2 run turtlebot3_navi_my multi_goals4_nav2
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True
#
#   #$ ros2 run turtlebot3_navi_my multi_goals4_cmd_vel
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=True
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
#from ament_index_python.packages import get_package_src_directory

from launch.actions import GroupAction


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_ros'), 'launch', 'config', 'rgbd.rviz'
    )

    bringup_dir = get_package_share_directory('nav2_bringup')
    config_rviz2=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),


    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_imu':qos,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'tf_delay':0.04,      # default 0.05  20[hz]
    }
    
    remappings=[
          ('odom', '/odom'),
          ('scan', '/scan'),
          # add by nishi
          ('map','/map')]

    return LaunchDescription([
        DeclareLaunchArgument('PC', default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2', default_value='false', description='Launch SBC (optional).'),

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        #DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('qos', default_value='1', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),

        DeclareLaunchArgument('rtabmapviz',default_value='false', description='Launch rtabmapviz (optional).'),
        DeclareLaunchArgument('rviz',default_value='false', description='Launch RVIZ (optional).'),

        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, description='Configuration path of rviz2.'),
        DeclareLaunchArgument('rviz_cfg2', default_value=config_rviz2, description='Configuration path of rviz2.'),

        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        # Nodes to launch
        # SLAM mode:

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'],
            namespace=LaunchConfiguration('namespace'),
            ), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'),
            ),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=[parameters],
            condition=IfCondition(LaunchConfiguration("rtabmapviz")),
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'),
            ),

        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg2")]],
            ),
    ])
