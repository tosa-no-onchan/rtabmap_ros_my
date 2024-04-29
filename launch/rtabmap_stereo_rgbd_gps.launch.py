# -*- coding: utf-8 -*-
#
# rtabmap_ros_my/launch/ratbmap_stero_rgbd_gps.launch.py
#
#  Rtabmap_ros with Stereo Camera (rgbd) and GPS Mapping or Acitve SLAM with 
#      robot_localization/navsat_transform_node and robot_localization/ekf_node
#
# foxbot_core3_r2.ino setting
#   same with foxbot_nav2_stereo_gps.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#  $ rosdep update && rosdep install --from-path src --ignore-src -y
#
# 2. set system clock sync of remote PC and SBC
#  http://192.168.1.46:8000/date?set=1
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
#  2) term2 start Camera, GPS, static_transform_publisher and others.
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py SBC:=true
#
#  3) term3
#   check topic
#     $ ros2 topic hz /odom
#   run rtabmap_ros on SBC
#     $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py PC:=true
#
# 5. Manual Control Mapping by telop
# 5.1 Remote PC /cmd_vel controll  -- Mapping
#  1) Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  2) drive_base include heart beat function.
#   $ ros2 run turtlebot3_navi_my drive_base
#
# 6. Active SLAM Mapping with navigation2 On SBC
#  6.1 run navigation2 on SBC
#  1) navigation2 dwa
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/nav2_params_ekf.yaml
#
#  1') navigation2 teb_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/teb_params_ekf.yaml
#
#  1'') navigation2 rpp_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/rpp_params_ekf.yaml
#
#  6.2 on Remote PC
#  1) Rviz2
#    $ ros2 launch nav2_bringup rviz_launch.py
#     or
#   $ ros2 launch rtabmap_ros_my rtabmap_stereo_rgbd_gps.launch.py PC2:=true
#
#  6.3 robot control #2  on SBC or Remote PC
#  1) On Remote PC Teleop keyboard drive
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#  or
#  2) On SBC C++ Program control include heart_beat function.
#   #$ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=False
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#
#   check
#   $ ros2 topic hz /fox_beat
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

    #config_rviz = os.path.join(
    #    get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    #)
    config_rviz = '/home/nishi/colcon_ws/src/rtabmap_ros_my/launch/config/rgbd.rviz'

    uvc_camera = get_package_share_directory('uvc_camera')
    stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')
    #gysfdmaxb_gps=get_package_share_directory('gysfdmaxb_gps')
    lc29h_gps_rtk=get_package_share_directory('lc29h_gps_rtk')

    rtabmap_parameters={
        "frame_id": "base_footprint",
        "subscribe_depth": False,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "subscribe_scan_cloud":False,
        "subscribe_stereo": False,
        "approx_sync": True,
        "queue_size": 10,
        "qos_image": qos,
        "qos_camera_info": qos,
        "qos_user_data": qos,
        # add for navigation2 by nishi
        'use_action_for_goal':True,
        #"Mem/IncrementalMemory":"True",
        #"Mem/InitWMWithAllNodes": "False",
        #'tf_delay':0.04545,      # 22[hz] default 0.05  20[hz]
        #'tf_delay':0.0625,      # 16[hz] default 0.05  20[hz]   PC and teb_local_planner is needed
        # add by nishi 2023.3.3
        #"Stereo/MaxDisparity": "800.0",
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
        ("odom", "/odom"),
        #("odom", "/odom_fox"),
        # publish
        # 'mapData'
        # 'mapGraph'
        ('map','/map'),
        ]


    return LaunchDescription([

        DeclareLaunchArgument('SBC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2',default_value='false', description='Launch SBC (optional).'),

        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        DeclareLaunchArgument('rviz',default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,description='Configuration path of rviz2.'),

        DeclareLaunchArgument('gps',default_value='true', description=''),

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


                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        #os.path.join(gysfdmaxb_gps,'launch', 'gysfdmaxb_gps.launch.py')
                        os.path.join(lc29h_gps_rtk,'launch', 'lc29h_gps_rtk.launch.py')
                    ),
                    launch_arguments={
                        #'rate':'6',
                        'rate':'1',
                        'device':'/dev/ttyUSB0',
                        'topicName':'/gps/fix'}.items(),
                    condition=IfCondition(LaunchConfiguration('gps')),
                ),


                Node(
                    package='rtabmap_sync', executable='stereo_sync', output="screen",
                    #condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
                    parameters=[{
                        "approx_sync": True,
                        #"approx_sync_max_interval": 0.01,
                        "approx_sync_max_interval": 0.05,
                        "queue_size": 10,
                        "qos": qos,
                        "qos_camera_info": 0}],
                    remappings=[
                        ("left/image_rect", '/left/image_rect_color'),
                        ("right/image_rect", '/right/image_rect_color'),
                        ("left/camera_info", '/left/camera_info'),
                        ("right/camera_info", '/right/camera_info'),
                        #("rgbd_image", '/rgbd_image')
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
                        #"qos": 0,
                        "qos": 1,
                    }],
                    remappings=[
                        ('disparity/image', '/disparity'),   #
                        ('disparity/camera_info', '/right/camera_info'),
                        ('cloud', '/cloudXYZ')],
                    namespace=LaunchConfiguration('namespace'),
                ),

                Node(
                    # https://yoshiaki-toyama.com/robot_localization/
                    # https://github.com/cra-ros-pkg/robot_localization/blob/foxy-devel/doc/navsat_transform_node.rst
                    package='robot_localization', executable='navsat_transform_node', name='navsat_transform_node', output='screen',
                    parameters=[{
                        "publish_filtered_gps": True,
                        "yaw_offset": 1.5707963,
                        "zero_altitude": True,
                    }],
                    remappings=[
                        # subscribe
                        ('gps/fix', '/gps/fix'), 
                        ('imu', '/imu'),
                        #('imu/data', '/imu_fox'),
                        ('odometry/filtered', '/odom_fox'),
                        # publish
                        ('odometry/gps', '/odom_gps'),
                        #('gps/filtered', '/gps/filtered'),
                        ],
                ),

                Node(
                    # https://github.com/cra-ros-pkg/robot_localization/blob/foxy-devel/launch/ekf.launch.py
                    package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
                    parameters=[os.path.join(get_package_share_directory("rtabmap_ros_my"), 'params','foxbot_core3', 'ekf.yaml')],
                    remappings=[
                        # subscribe
                        ('wheel', '/odom_fox'), 
                        ('odom_gps', '/odom_gps'),
                        ('imu0', '/imu'),
                        #('imu/data', '/imu_fox'),
                        # publish
                        #('odometry/filtered', '/odom_fusion'),
                        ('odometry/filtered', '/odom'),
                        ],
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
