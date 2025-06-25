# -*- coding: utf-8 -*-
#
# ROS2 jazzy
# SBC : Orange pi 5
#       ubuntu 22.04
#
# rtabmap_ros_my/launch/foxbot_bringup.launch.py
#   -> start
#    1) micro_ros_agent   
#    2) rtabmap_ros_my foxbot_nav2_stereo_gps.launch
#    3) Map Server map load
#    4) navigation2
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# 2. set system clock sync of remote PC and SBC
#  http://192.168.1.39:8000/date?set=1
#   or
#  On SBC
#   s sudo systemctl stop chronyd
#   $ sudo chronyd -q 'server 192.168.1.170 iburst'
#   $ sudo systemctl start chronyd
#
# 3. disable firewall on remote PC
#  $ sudo ufw disable
#
#  Run :
#  1. On SBC
#   $ ros2 launch rtabmap_ros_my foxbot_bringup.launch.py [params_file:=FUll Path to params.yaml]
#
#  2. On PC, Rviz
#   $ ros2 launch nav2_bringup rviz_launch.py
#
#  3. On SBC, C++ Program controll
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=False
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#
#  4. C++ Auto Mower [localization and  navigation]
#   $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=False threshold:=200.0 all_nav2:=true [plann_test:=true] 
#
#
# reffer
#  https://ar-ray.hatenablog.com/entry/2023/06/25/204119
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


#from launch.actions import (EmitEvent, ExecuteProcess, IncludeLaunchDescription,
#                            LogInfo, RegisterEventHandler, TimerAction)

def generate_launch_description():


    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    log_level = LaunchConfiguration('log_level')


    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')

    SBC = LaunchConfiguration('SBC')

    rtabmap_ros_my_launch_file_dir = os.path.join(get_package_share_directory('rtabmap_ros_my'), 'launch')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    declare_use_param_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_nav2', 'oak-d_rpp_params.yaml'),
        #default=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_nav2', 'nav2_params.yaml'),
        )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_SBC_cmd = DeclareLaunchArgument(
        'SBC',
        default_value='true',
        description='SBC run',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    #param_dir = DeclareLaunchArgument(
    #        'params_file',
    #        #default_value=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_nav2', 'rpp_params.yaml'),
    #        default_value=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_nav2', 'nav2_params.yaml'),
    #        description='Full path to the ROS2 parameters file to use'),

    # start foxbot_core_r2 comuncation
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',output='screen',
        arguments=['serial','--dev','/dev/ttyS0','-b','1000000'],
        #parameters=[{'use_sim_time': use_sim_time}],
        )
    
    bringup_cmd_group = GroupAction(
        [
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                #parameters=[configured_params, {'autostart': autostart}],
                parameters=[{'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            # Map Server map load
            # start rtabmap_ros_my localization.launch.py
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([rtabmap_ros_my_launch_file_dir, '/localization.launch.py']),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': params_file
                        }.items(),
                ),
            # start rtabmap_ros_my foxbot_nav2_oak-d_depth_gps.launch.py
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([rtabmap_ros_my_launch_file_dir, '/foxbot_nav2_oak-d_depth_gps.launch.py']),
                    launch_arguments={
                        #'SBC': 'true',
                        'SBC': SBC,
                        #'use_composition': 'True',
                        #'map': map_dir,
                        #'use_sim_time': use_sim_time,
                        #'params_file': param_dir
                        }.items(),
                ),
            # navigation2
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': params_file
                        }.items(),
                ),
        ]
    )

    #return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),
    #])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_SBC_cmd)
    ld.add_action(declare_use_param_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(micro_ros_agent_node)
    ld.add_action(bringup_cmd_group)
    return ld