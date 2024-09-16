# -*- coding: utf-8 -*-
#
# ROS2 humble
# SBC : Orange pi 5
#       ubuntu 22.04
#
# rtabmap_ros_my/launch/foxbot_nav2_stereo_gps.launch.py
#
#  rbot navigation with robot_localization/ekf_node, Stereo Camera (rgbd) and GPS.
#  
# foxbot_core3_r2.ino setting
#  1)#define USE_ODOM_FOX   -> odom_fox
#  2)bool use_tf_static=false;
#  3)bool use_imu_pub=true;
#
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
# 4 run
# 4.1 run on SBC (Orange pi 5)
#  1) term1  -> add group               group
#   $ sudo chmod 777 /dev/ttyS0     ->  dialout
#   $ sudo chmod 777 /dev/ttyUSB0   ->  dialout
#   $ sudo chmod 777 /dev/video0    ->  video
#   $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 1000000 [-v6]
#
#  2) term2
#   $ ros2 launch rtabmap_ros_my foxbot_nav2_stereo_gps.launch.py SBC:=true
#
#  Map Server map load
#  3) term3
#   $ ros2 launch rtabmap_ros_my localization.launch.py
#
#  navigation2 
#  4) navigation2 dwb , params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_nav2/nav2_params.yaml
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_nav2/nav2_params.yaml
#
#  4') navigation2 teb_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_nav2/teb_params.yaml
#
#  4'') navigation2 rpp_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_nav2/rpp_params.yaml
#
# 5. Rviz2 on Remote PC
#   1)
#    $ ros2 launch nav2_bringup rviz_launch.py
#   or
#   2) PC2:=true 
#    $ ros2 launch rtabmap_ros_my foxbot_nav2_stereo_gps.launch.py PC2:=true
#
# 6. run  on remote PC or SBC
#  1) C++ Program control
#   #$ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=False
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=False
#
#  [multi_goals4_nav2-1] GetMap::get(): 99 error が出たら、SBC 上で実行する。 2024.2.10
#
# append.
# how to map save
# ros2 run nav2_map_server map_saver_cli -f ~/map/my_map --ros-args -p save_map_timeout:=10000.0
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
from nav2_common.launch import RewrittenYaml

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
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )
    config_rviz_fox_nav2 = os.path.join(
        get_package_share_directory('rtabmap_ros_my'),'launch','config','fox-nav2.rviz'
    )

    uvc_camera = get_package_share_directory('uvc_camera')
    stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')
    #gysfdmaxb_gps=get_package_share_directory('gysfdmaxb_gps')
    #lc29h_gps=get_package_share_directory('lc29h_gps')
    lc29h_gps_rtk=get_package_share_directory('lc29h_gps_rtk')

    rtabmap_parameters={
        "subscribe_depth": False,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "subscribe_scan_cloud":False,
        "subscribe_stereo": False,
        "frame_id": "base_footprint",
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
        "Stereo/MaxDisparity": "800.0",
    }
    rtabmap_remappings=[
        # subscribe
        ("rgb/image","rgbd_image/compressed"),
        ("odom", "/odom_fox"),
        # publish
        ('map','/map')]

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time',default_value='false', description='sim time.'),
        DeclareLaunchArgument('SBC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('MAP',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('NAV2',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),
        DeclareLaunchArgument('rviz',default_value='true', description='Launch RVIZ (optional).'),
        #DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,description='Configuration path of rviz2.'),
        DeclareLaunchArgument('rviz_cfg_fox_nav2', default_value=config_rviz_fox_nav2,description='Configuration path of rviz2.'),
        DeclareLaunchArgument('gps',default_value='true', description=''),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join('/','home','nishi','map','my_map.yaml'),
            description='Full path to map yaml file to load'),

        # /home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_core3/nav2_params.yaml
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('rtabmap_ros_my'), 'params','foxbot_nav2', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        GroupAction(
            [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                    output="screen",
                ),
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
                    # x y z yaw pitch roll
                    #arguments=['0', '0', '0.193', '-1.5707963267948966', '0', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    # -0.6[degre] 下向き の補正
                    #arguments=['0', '0', '0.193', '-1.5707963267948966', '-0.010471975511965976', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    # -0.7[degre] 下向き の補正
                    #arguments=['0', '0', '0.193', '-1.5707963267948966', '-0.012217304763960306', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    #arguments=['0', '0', '0.193', '0', '-0.012217304763960306', '0', 'base_link', 'stereo_camera'],
                    arguments=['0.038', '0', '0.193', '0', '-0.012217304763960306', '0', 'base_link', 'stereo_camera'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link2',
                    #arguments=['0', '0', '0', '0', '0', '0', 'stereo_camera', 'source_frame'],
                    arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'stereo_camera', 'source_frame'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link_right',
                    arguments=['0.03', '0', '0', '0', '0', '0', 'source_frame', 'right_camera'],
                    output="screen",
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_base_link_left',
                    arguments=['-0.03', '0', '0', '0', '0', '0', 'source_frame', 'left_camera'],
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
                    launch_arguments={'left/device': '/dev/video0','qos': '1' , 'intra':'False', 'trace':'True', 'fps': '15' }.items(),
                    #launch_arguments={'left/device': '/dev/video0','qos': '1' , 'intra':'False', 'trace':'True', 'fps': '20' }.items(),
                    # publish
                    # /left/camera_info
                    # /left/image_raw
                    # /parameter_events
                    # /right/camera_info
                    # /right/image_raw
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(stereo_image_proc,'launch', 'stereo_image_proc.launch.py')
                    ),
                    launch_arguments={'left_namespace':'left' ,'right_namespace':'right'}.items(),
                    # publish
                    # /left/image_rect
                    # /left/image_rect_color
                    # /right/image_rect
                    # /right/image_rect_color
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(lc29h_gps_rtk,'launch', 'lc29h_gps_rtk.launch.py')
                    ),
                    launch_arguments={
                        #'rate':'6',         # best fit value, use this value 6 2024.2.22
                        'rate':'1',             # test for LC92H(DA) RTK 1[Hz]
                        'device':'/dev/ttyUSB0',
                        'topicName':'/gps/fix'}.items(),
                    condition=IfCondition(LaunchConfiguration('gps')),
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
                        "qos": 0,
                        #"qos": 1,
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
                        #"yaw_offset": 1.5707963,
                        "yaw_offset": 1.47735,
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
                    parameters=[os.path.join(get_package_share_directory("rtabmap_ros_my"), 'params','foxbot_nav2', 'ekf.yaml')],
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
                #Node(
                #    condition=UnlessCondition(localization),
                #    package='rtabmap_ros', executable='rtabmap', output='screen',
                #    parameters=[rtabmap_parameters],
                #    remappings=rtabmap_remappings,
                #    arguments=['-d'],
                #    namespace=LaunchConfiguration('namespace'),
                #    ),

                # Localization mode:
                #Node(
                #    condition=IfCondition(localization),
                #    package='rtabmap_ros', executable='rtabmap', output='screen',
                #    parameters=[rtabmap_parameters,
                #        {'Mem/IncrementalMemory':'False',
                #        'Mem/InitWMWithAllNodes':'True'}],
                #    remappings=rtabmap_remappings,
                #    namespace=LaunchConfiguration('namespace'),
                #    ),
                #
                # https://index.ros.org/p/nav2_map_server/
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    #parameters=[configured_params],
                    parameters=[{'yaml_filename': map_yaml_file}]
                    #remappings=remappings
                    ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                                {'autostart': True},
                                {'node_names': ['map_server']}]
                    )
            ],
            condition=IfCondition(LaunchConfiguration('MAP')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),
        GroupAction(
            [
                Node(
                    package='rviz2', executable='rviz2', output='screen',
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    #arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
                    arguments=[["-d"], [LaunchConfiguration("rviz_cfg_fox_nav2")]]),
            ],
            condition=IfCondition(LaunchConfiguration('PC2')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),
        GroupAction(
            #   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/foxbot_nav2/nav2_params.yaml
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                       os.path.join(get_package_share_directory('nav2_bringup'),'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time') ,'params_file':params_file}.items(),
                ),
            ],
            condition=IfCondition(LaunchConfiguration('NAV2')),
        ),
    ])