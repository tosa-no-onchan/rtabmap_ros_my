# -*- coding: utf-8 -*-
#
# rtabmap_ros_my/launch/rtabmap_oak-d_rgb_depth.launch.py
#  base: rtabmap_ros_my/launch/ratbmap_stero_rgbd.launch.py
#
#  Rtabmap_ros with oak-d depth Mapping or Acitve SLAM
#
#  foxbot_core3_r2.ino で、
#   1)#define USE_ODOM_FOX   -> /odom_fox publish を有効にする。
#   2)bool use_tf_static=true;   -> /tf を、 publish する。
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
# 4.1 run on SBC (Orange Pi5 armibian)
#  1) term1  -> add group               group
#   $ sudo chmod 777 /dev/ttyS0     ->  dialout
#   $ sudo chmod 777 /dev/ttyUSB0   ->  dialout
#   $ sudo chmod 777 /dev/video0    ->  video
#
#   $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 1000000 [-v6]
#
#  2) term2 Camera
#   $ ros2 launch rtabmap_ros_my rtabmap_oak-d_rgb_depth.launch.py SBC:=true
#    or with rtabmap_ros
#   $ ros2 launch rtabmap_ros_my rtabmap_oak-d_rgb_depth.launch.py SBC:=true PC:=true
#
# 4.3 run rtabmap_ros on remote PC or SBC
#  1) term3
#   $ ros2 launch rtabmap_ros_my rtabmap_oak-d_rgb_depth.launch.py PC:=true
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
#  1'') navigation2 rpp_planner
#   #$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws-jazzy/src/rtabmap_ros_my/params/foxbot_core3/oak-d_rpp_params_ekf.yaml
#   $ ros2 launch rtabmap_ros_my navigation.launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws-jazzy/src/rtabmap_ros_my/params/foxbot_core3/oak-d_rpp_params_ekf.yaml
#
#  5.3 Rviz2 on Remote PC
#  1) Rviz2
#    $ ros2 launch nav2_bringup rviz_launch.py
#     or
#   $ ros2 launch rtabmap_ros_my rtabmap_oak-d_rgb_depth.launch.py PC2:=true
#
#
#  5.4 robot control #2  on SBC or Remote PC
#  1)  Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  1') C++ Program controll
#   #$ ros2 run turtlebot3_navi_my multi_goals4_nav2
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#
# append.
# how to map save ,on Remote PC OK
# ros2 run nav2_map_server map_saver_cli -f ~/map/my_map --ros-args -p save_map_timeout:=10000.0

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

    #uvc_camera = get_package_share_directory('uvc_camera')
    #stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')
    depthai_ros_my=get_package_share_directory('depthai_ros_my')

    auto_exp       = LaunchConfiguration('auto_exp', default = True)
    sensIso        = LaunchConfiguration('sensIso', default = 1200)
    expTime        = LaunchConfiguration('expTime', default = 28500)


    rtabmap_parameters={
        "frame_id": "base_footprint",
        "subscribe_depth": True,
        "subscribe_rgbd": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud":False,
        "subscribe_stereo": False,
        "approx_sync": True,
        "queue_size": 15,
        #"queue_size": 30,
        #"sync_queue_size": 15,  # add by nishi 2024.9.10
        "sync_queue_size": 20,  # add by nishi 2025.3.19
        #"topic_queue_size": 15, # add by nishi 2024.9.10
        "topic_queue_size": 20, # add by nishi 2025.3.19
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
        ("rgb/image","/color/video/image"),
        ("rgb/camera_info","/color/video/camera_info"),
        ("depth/image","/stereo/depth"),
        #("odom", "/odom"),      # foxbot_core3_r2.ino で、 /odom を、publish する。
        ("odom", "/odom_fox"),    # foxbot_core3_r2.ino で、 /odom_fox を、publish する。
        # publish
        ('map','/map'),
        ]

    rtabmap_rviz_remappings=[
        # subscribe
        ("rgb/image","/rtabmap/rgbd_image"),
        ('rgb/camera_info', '/right/camera_info'),
        #("odom", "/odom"),
        ("odom", "/odom_fox"),
        #('depth/image', '/rtabmap/depth/image')
        # publish
        # 'mapData'
        # 'mapGraph'
        #('map','/map'),
        ]

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time',default_value='false', description='sim time.'),
        DeclareLaunchArgument('SBC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2',default_value='false', description='Launch SBC (optional).'),

        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        DeclareLaunchArgument('rviz',default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,description='Configuration path of rviz2.'),

        DeclareLaunchArgument('rtabmap_rviz',default_value='false', description='Launch rtabmap_ros RVIZ (optional).'),

        DeclareLaunchArgument('rate',default_value='10', description=''),
        DeclareLaunchArgument('queue_size',default_value='2', description=''),
        DeclareLaunchArgument('rgb2grey',default_value='false', description=''),

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
                    # python3
                    # import math
                    # math.radians(1)    1[degree] -> 0.017453292519943295[radian]
                    # x,y,z,roll,pitch.yaw
                    # pitch 補正は、pitch > 0 が、前が下向く pitch < 0 前が上向く
                    #arguments=['0.038', '0', '0.193', '0', '-0.012217304763960306', '0', 'base_link', 'oak'],
                    # changed by nishi 2025.3.24
                    #arguments=['0.038', '0', '0.193', '0', '0', '0', 'base_link', 'oak'], # これが、水平。
                    # 0.9 - 1.1[degree]下向きで、1[cm] 下げるが、基準か
                    # カメラ取り付け角 下 0.9[dgree] 0.015707963267948967[radian] 下向きのようなので、下向きを知らせる pitch > 0 が、下向き
                    #arguments=['0.038', '0', '0.193', '0', '0.015707963267948967', '0', 'base_link', 'oak'],       # 完走 OK。コース補正が少ない。一発でOK
                    # カメラ取り付け角 下 1.1[dgree] 0.019198621771937627[radian] 下向きのようなので、下向きを知らせる pitch > 0 が、下向き
                    #arguments=['0.038', '0', '0.193', '0', '0.019198621771937627', '0', 'base_link', 'oak'],       # まだ!!
                    # 取り付け高さ: 1cm下げる。カメラ取り付け角 下 1.1[dgree] 0.019198621771937627[radian] 下向きのようなので、下向きを知らせる pitch > 0 が、下向き
                    #arguments=['0.038', '0', '0.183', '0', '0.019198621771937627', '0', 'base_link', 'oak'],       # まあまあOK。 start地点が、下に潜る。コース補正がすくないか!!
                    # 取り付け高さ: 2cm下げる。カメラ取り付け角 下 1.1[dgree] 0.019198621771937627[radian] 下向きのようなので、下向きを知らせる pitch > 0 が、下向き
                    #arguments=['0.038', '0', '0.173', '0', '0.019198621771937627', '0', 'base_link', 'oak'],       # まあまあ OKか。コース補正が少しある。
                    # 取り付け高さ: 1cm下げる。カメラ取り付け角 下 1.2[dgree] 0.020943951023931952[radian] 下向きのようなので、下向きを知らせる pitch > 0 が、下向き
                    arguments=['0.038', '0', '0.183', '0', '0.020943951023931952', '0', 'base_link', 'oak'],       # まあまあOKか。start地点が、下に潜る。戻りが近いか!!コース補正動作がある。
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link2',
                    #arguments=['0', '0', '0', '0', '0', '0', 'stereo_camera', 'source_frame'],
                    #arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'stereo_camera', 'source_frame'],
                    arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'oak', 'oak-d-base-frame'],
                    output="screen",
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link3',
                    arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak-d_frame'],
                    output="screen",
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_center',
                    arguments=['0', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_rgb_camera_optical_frame'],
                    output="screen",
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_right',
                    arguments=['0.0375', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_right_camera_optical_frame'],
                    output="screen",
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_left',
                    arguments=['-0.0375', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_left_camera_optical_frame'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='imu_base_link',
                    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='gps_base_link',
                    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link'],
                    output="screen",
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(depthai_ros_my , 'launch', 'oak-d_rgb_stereo_node.launch.py')
                    ),
                    launch_arguments={'rate': LaunchConfiguration('rate'),
                                      'queue_size': LaunchConfiguration('queue_size'),
                                      'rgb2grey': LaunchConfiguration('rgb2grey'),
                                      'auto_exp': auto_exp,
                                      'sensIso': sensIso,
                                      'expTime': expTime,
                                      }.items(),
                    # publish
                    # /color/video/camera_info
                    # /color/video/image
                    # /color/video/image/compressed
                    # /color/video/image/compressedDepth
                    # /color/video/image/theora
                    # /stereo/camera_info
                    # /stereo/depth
                    # /stereo/depth/compressed
                    # /stereo/depth/compressedDepth
                    # /stereo/depth/theora
                ),

                Node(
                    # https://github.com/ros-perception/image_pipeline/tree/foxy/depth_image_proc/src
                    # camera_info (sensor_msgs/CameraInfo) 
                    # image_rect (sensor_msgs/Image) 
                    package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                    parameters=[{
                        "decimation": 4,
                        #"voxel_size": 0.0,
                        # changed by nishi 2024.5.9
                        "voxel_size": 0.05,
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
                        #('disparity/image', '/disparity'),   #
                        #('disparity/camera_info', '/right/camera_info'),
                        ('depth/camera_info','/stereo/camera_info'),
                        ('depth/image','/stereo/depth'),
                        ('cloud', '/cloudXYZ')],
                    # subscribe
                    #  depth/camera_info
                    #  depth/image
                    #  ------
                    #  disparity/camera_info
                    #  disparity/image
                    # publish
                    #  cloud
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

                Node(
                    package='rtabmap_viz', executable='rtabmap_viz', output='screen',
                    condition=IfCondition(LaunchConfiguration("rtabmap_rviz")),
                    parameters=[rtabmap_parameters,{'approx_sync':'True'}],
                    remappings=rtabmap_rviz_remappings,
                    namespace=LaunchConfiguration('namespace'),
                    ),

            ],
            condition=IfCondition(LaunchConfiguration('PC2')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),

    ])
