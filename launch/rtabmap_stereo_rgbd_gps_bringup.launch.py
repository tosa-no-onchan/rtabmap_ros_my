# -*- coding: utf-8 -*-
#
# ROS2 humble
# SBC : Orange pi 5
#       ubuntu 22.04
#
# rtabmap_ros_my/launch/ratbmap_stero_rgbd_gps_bringup.launch.py
#   -> start
#    1) micro_ros_agent
#    2) rtabmap_ros_my ratbmap_stero_rgbd_gps.launch SBC:=true
#    2) rtabmap_ros_my ratbmap_stero_rgbd_gps.launch PC:=true
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
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd_gps_bringup.launch.py 
#
#   nav2 with rpp start
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/rpp_params_ekf.yaml
#
#  2. On PC, Rviz
#   $ ros2 launch nav2_bringup rviz_launch.py
#     or
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py PC2:=true
#
#  3. On SBC, C++ Program controll
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=False
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#
#  4. C++ Auto Map
#   $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py use_sim_time:=False 
#
#
# reffer
#  https://ar-ray.hatenablog.com/entry/2023/06/25/204119
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


#from launch.actions import (EmitEvent, ExecuteProcess, IncludeLaunchDescription,
#                            LogInfo, RegisterEventHandler, TimerAction)

def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #sbc = LaunchConfiguration('SBC', default='true')

    rtabmap_ros_my_launch_file_dir = os.path.join(get_package_share_directory('rtabmap_ros_my'), 'launch')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    param_dir = LaunchConfiguration(
        'params_file',
        #default=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_core3', 'rpp_params_ekf.yaml'),
        default='/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/rpp_params_ekf.yaml',
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
    #micro_ros_delay = TimerAction(period=5.0, actions=[micro_ros_agent_node])

    # start rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py SBC:=true
    rtabmap_stereo_sbc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rtabmap_ros_my_launch_file_dir, '/rtabmap_stereo_rgbd_gps.launch.py']),
            launch_arguments={
                'SBC': 'true',
                }.items(),
        )
    rtabmap_stereo_sbc_delay = TimerAction(period=2.0, actions=[rtabmap_stereo_sbc])

    # start rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py PC:=true
    rtabmap_stereo_pc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rtabmap_ros_my_launch_file_dir, '/rtabmap_stereo_rgbd_gps.launch.py']),
            launch_arguments={
                'PC': 'true',
                }.items(),
        )
    rtabmap_stereo_pc_delay = TimerAction(period=5.0, actions=[rtabmap_stereo_pc])

    # navigation2 with rpp_planner
    nav2_rpp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir
                }.items(),
        )
    nav2_rpp_delay = TimerAction(period=30.0, actions=[nav2_rpp])

    #return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),
    #])

    ld.add_action(micro_ros_agent_node)
    ld.add_action(rtabmap_stereo_sbc_delay)
    #ld.add_action(rtabmap_stereo_sbc)
    #ld.add_action(nav2_rpp_delay)
    #ld.add_action(nav2_rpp)
    ld.add_action(rtabmap_stereo_pc_delay)
    #ld.add_action(rtabmap_stereo_pc)
    return ld