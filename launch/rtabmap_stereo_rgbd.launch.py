# -*- coding: utf-8 -*-
#
# rtabmap_ros_my/launch/ratbmap_stero_rgbd.launch.py
#
#  Rtabmap_ros with Stereo Camera (rgbd) Mapping or Acitve SLAM
#
#  foxbot_core3_r2.ino で、
#    1) /odom or /odom_fox を、publish する。
#    2) /tf を、publish する。
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# 2. set system clock sync of remote PC and SBC
#  http://192.168.1.42:8000/date?set=1
#   or
#  On SBC
#   s sudo systemctl stop chronyd
#   $ sudo chronyd -q 'server 192.168.1.170 iburst'
#   $ sudo systemctl start chronyd
#
# 3. disable firewall on remote PC
#  $ sudo ufw disable
#
# 4 run
# 4.1 run heart_beat on remote PC
#  $ ros2 run foxbot_tool heart_beat
#   check
#  $ ros2 topic hz /fox_beat
#
# 4.2 run on SBC (Jetson Nano 2G)
#  1) term1
#   $ sudo chmod 777 /dev/ttyS0
#   $ sudo chmod 777 /dev/ttyUSB0
#   $ sudo chmod 777 /dev/video0
#   $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 1000000 [-v6]
#
#  2) term2
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd.launch.py SBC:=true
#    or with rtabmap_ros
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd.launch.py SBC:=true PC:=true
#
# 4.3 run rtabmap_ros on remote PC or SBC
#  1) term3
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd.launch.py PC:=true
#
#
# 5. Remote control
# 5.1 Remote PC /cmd_vel controll  -- Mapping
#  1) Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  2) drive_base
#   $ ros2 run turtlebot3_navi_my drive_base
#
# 5.2 Remote PC or SBC / navigation2  ---  Acitve SLAM
#  1) check
#   $ ros2 topic hz /cloudXYZ
#
#  2) navigation2 dwa
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/nav2_params.yaml
#   or
#  2') navigation2 teb_local_planner
#   $ ros2 launch nav2_bringup navigation.launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/teb_params.yaml
#
#  5.3 Rviz2 
#  1) Rviz2 on Remote PC
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd.launch.py PC2:=true
#    or
#   $ ros2 launch nav2_bringup rviz_launch.py
#
#  5.4 control Robot
#  1)  Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  1') C++ Program controll
#   #$ ros2 run turtlebot3_navi_my multi_goals4_nav2
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#

import os

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction

from launch.conditions import IfCondition, UnlessCondition


from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression

from typing import Text


class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else


def generate_launch_description():

    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')


    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    uvc_camera = get_package_share_directory('uvc_camera')
    stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')

    rtabmap_parameters={
        "frame_id": "base_footprint",
        "subscribe_depth": False,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "subscribe_scan_cloud":False,
        "subscribe_stereo": False,
        "approx_sync": True,
        #"queue_size": 10,
        "queue_size": 30,
        "qos_image": qos,
        "qos_camera_info": qos,
        "qos_user_data": qos,
        # add for navigation2 by nishi
        'use_action_for_goal':True,
        #"Mem/IncrementalMemory":"True",
        #"Mem/InitWMWithAllNodes": "False",
        'tf_delay':0.0625,      # default 0.05  20[hz]
        # add by nishi 2024.4.28
        'Grid/RangeMax':'2.0',    # add by nishi 
        #'Grid/MaxGroundHeight':'0.05',    # add by nishi 2024.3.12 Very Good!! 3[M] 先の床が障害物になるのを防ぐ
        #'Grid/MaxGroundHeight':'0.07',    # add by nishi 2024.3.12 Very Good!! 3[M] 先の床が障害物になるのを防ぐ
        'Grid/MaxGroundHeight':'0.1',    # add by nishi 2024.3.12 Very Good!! 3[M] 先の床が障害物になるのを防ぐ
        'Grid/MaxObstacleHeight':'0.7',   # add by nishi 2024.3.11 Needed
    }
    rtabmap_remappings=[
        # subscribe
        ("rgb/image","rgbd_image/compressed"),
        ("odom", "/odom"),      # foxbot_core3_r2.ino で、 /odom を、publish する。
        #("odom", "/odom_fox"),    # foxbot_core3_r2.ino で、 /odom_fox を、publish する。
        # publish
        ('map','/map')]


    return LaunchDescription([

        DeclareLaunchArgument('SBC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2',default_value='false', description='Launch SBC (optional).'),

        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        DeclareLaunchArgument('rviz',default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,description='Configuration path of rviz2.'),

        GroupAction(
            [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link',
                    arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link',
                    arguments=['0', '0', '0.193', '-1.5707963267948966', '0', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link2',
                    arguments=['0', '0', '0', '0', '0', '0', 'stereo_camera', 'source_frame'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link3',
                    arguments=['0', '0', '0', '0', '0', '0', 'source_frame', 'camera'],
                    output="screen",
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(uvc_camera, 'launch', 'single_stereo_node.launch.py')
                    ),
                    #launch_arguments={'left/device': '/dev/video0'}.items(),
                    #launch_arguments={'left/device': '/dev/video0','qos': '0', 'intra':'False', 'trace':'True' }.items(),
                    launch_arguments={'left/device': '/dev/video0','qos': '0', 'intra':'False', 'trace':'True', 'fps': '15' }.items(),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(stereo_image_proc,'launch', 'stereo_image_proc.launch.py')
                    ),
                    launch_arguments={'left_namespace':'left' ,'right_namespace':'right'}.items(),
                ),


                Node(
                    package='rtabmap_sync', executable='stereo_sync', output="screen",
                    #condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
                    parameters=[{
                        "approx_sync": True,
                        #"approx_sync_max_interval": 0.01,
                        "approx_sync_max_interval": 0.05,
                        #"approx_sync_max_interval": 0.07,
                        "queue_size": 10,
                        "qos": qos,
                        "qos_camera_info": 0}],
                    remappings=[
                        ("left/image_rect", '/left/image_rect_color'),      # subscribe
                        ("right/image_rect", '/right/image_rect_color'),    # subscribe
                        ("left/camera_info", '/left/camera_info'),          # subscribe
                        ("right/camera_info", '/right/camera_info'),        # subscribe
                        #("rgbd_image", '/rgbd_image')                      # publish
                        ],
                    namespace=LaunchConfiguration('namespace')
                ),

                Node(
                    # https://github.com/ros-perception/image_pipeline/tree/foxy/depth_image_proc/src
                    # camera_info (sensor_msgs/CameraInfo) 
                    # image_rect (sensor_msgs/Image) 
                    package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                    parameters=[{
                        "decimation": 4,
                        "voxel_size": 0.0,
                        #"voxel_size": 0.05,
                        "approx_sync": True,
                        #"exact_sync": True,
                        #"approx_sync_max_interval": 0.1 ,
                        #"approx_sync_max_interval": 0.2 ,
                        "approx_sync_max_interval": 0.5 ,
                        #"approx_sync_max_interval": 0.7 ,
                        #"qos": 0,
                        "qos": 1,
                    }],
                    remappings=[
                        ('disparity/image', '/disparity'),   # subscribe
                        ('disparity/camera_info', '/right/camera_info'),    # subscribe
                        ('cloud', '/cloudXYZ')],
                    namespace=LaunchConfiguration('namespace'),
                ),


            ],
            condition=IfCondition(LaunchConfiguration('SBC')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('SBC'), "' == 'true'"])),
        ),

        GroupAction(
            [
                # SLAM mode:
                Node(
                    condition=UnlessCondition(localization),
                    package='rtabmap_slam', executable='rtabmap', output='screen',
                    parameters=[rtabmap_parameters],
                    remappings=rtabmap_remappings,
                    arguments=['-d'],
                    namespace=LaunchConfiguration('namespace'),
                    ),

                # Localization mode:
                Node(
                    condition=IfCondition(localization),
                    package='rtabmap_slam', executable='rtabmap', output='screen',
                    parameters=[rtabmap_parameters,
                        {'Mem/IncrementalMemory':'False',
                        'Mem/InitWMWithAllNodes':'True'}],
                    remappings=rtabmap_remappings,
                    namespace=LaunchConfiguration('namespace'),
                    ),


            ],
            condition=IfCondition(LaunchConfiguration('PC')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),

        GroupAction(
            [
                Node(
                    package='rviz2', executable='rviz2', output='screen',
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),

            ],
            condition=IfCondition(LaunchConfiguration('PC2')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),

    ])
