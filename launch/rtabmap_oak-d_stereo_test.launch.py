# -*- coding: utf-8 -*-
#
# rtabmap_ros_my/launch/rtabmap_oak-d_stereo_test.launch.py
#
#  Rtabmap_ros with Stereo Camera (non rgbd) test
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# 3. disable firewall on remote PC
#  $ sudo ufw disable
#
# 4 run
#  1) term2 start Camera, static_transform_publisher , odom and others.
#   Stereo Test
#     $ ros2 launch rtabmap_ros_my rtabmap_oak-d_stereo_test.launch.py SBC:=true rate:=15 queue_size:=2 STEREO_TEST:=true [mode:=disparity]
#   Depth Test
#     $ ros2 launch rtabmap_ros_my rtabmap_oak-d_stereo_test.launch.py SBC:=true rate:=15 queue_size:=2 DEPTH_TEST:=true [mode:=disparity]
#
#  2) term3
#   run rtabmap_ros on SBC
#   Stereo Test
#    $ ros2 launch rtabmap_ros_my rtabmap_oak-d_stereo_test.launch.py PC_STEREO_TEST:=true
#   Depth Test
#    $ ros2 launch rtabmap_ros_my rtabmap_oak-d_stereo_test.launch.py PC_DEPTH_TEST:=true
#
#
# 5 run on remote PC Rviz2
#  1) Rviz2
#    $ ros2 launch nav2_bringup rviz_launch.py
#     or
#    $ ros2 launch rtabmap_ros_my rtabmap_oak-d_stereo_test.launch.py PC2:=true
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
        get_package_share_directory('rtabmap_ros_my'), 'launch', 'config', 'rgbd.rviz'
    )

    #uvc_camera = get_package_share_directory('uvc_camera')
    #stereo_image_proc = get_package_share_directory('stereo_image_proc')        # image_pipeline
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')
    depthai_ros_my=get_package_share_directory('depthai_ros_my')
    lc29h_gps_rtk=get_package_share_directory('lc29h_gps_rtk')

    rtabmap_parameters={
        "frame_id": "base_footprint",
        #"subscribe_depth": False,
        "subscribe_rgbd": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud":False,
        #"subscribe_stereo": True,
        "approx_sync": True,
        "queue_size": 10,
        "qos_image": qos,
        "qos_camera_info": qos,
        "qos_user_data": qos,
        # add for navigation2 by nishi
        'use_action_for_goal':True,
        #"Mem/IncrementalMemory":"True",
        #"Mem/InitWMWithAllNodes": "False",
        # add by nishi 2023.3.3
        "Stereo/MaxDisparity": "9000.0",
        'Grid/RangeMax': '3.0',    # add by nishi 
        'Grid/MaxGroundHeight': '0.12',    # add by nishi 2024.3.12 Very Good!! 3[M] 先の床が障害物になるのを防ぐ
        'Grid/MaxObstacleHeight': '0.7',   # add by nishi 2024.3.11 Needed
        'Rtabmap/TimeThr': '700.0',
        'RGBD/OptimizeMaxError': '3.4',    # add by nishi 2024.5.1
        'Odom/FilteringStrategy': '1',     # add by nishi 2024.5.1 0=No filtering 1=Kalman filtering 2=Particle filtering
    }
    rtabmap_remappings_stereo=[
        # subscribe
        #("rgb/image","rgbd_image/compressed"),
        #("left/image_rect", '/left/image_rect_color'),
        ("left/image_rect", '/left/image_rect'),
        #("left/image_rect", '/left/image_rect/compressed'),
        #("right/image_rect", '/right/image_rect_color'),
        ("right/image_rect", '/right/image_rect'),
        #("right/image_rect", '/right/image_rect/compressed'),
        ("left/camera_info", '/left/camera_info'),
        ("right/camera_info", '/right/camera_info'),
        #("odom", "/odom_fox"),
        ("odom", "/odom"),
        # publish
        ('map','/map')]

    rtabmap_remappings_desp=[
        # subscribe
        ("rgb/image","/color/video/image"),
        #("rgb/image","/color/video/image/compressed"),      # not suport
        ("rgb/camera_info","/color/video/camera_info"),
        ("depth/image","/stereo/depth"),
        #("odom", "/odom_fox"),
        ("odom", "/odom"),
        # publish
        ('map','/map')]

    return LaunchDescription([

        DeclareLaunchArgument('SBC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC',default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC2',default_value='false', description='Launch SBC (optional).'),

        DeclareLaunchArgument('STEREO_TEST',default_value='false', description='Rect image'),
        DeclareLaunchArgument('DEPTH_TEST',default_value='false', description='Rect image'),

        DeclareLaunchArgument('PC_STEREO_TEST',default_value='false', description='Rect image'),
        DeclareLaunchArgument('PC_DEPTH_TEST',default_value='false', description='Rect image'),

        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        DeclareLaunchArgument('rgbd',default_value='false', description=''),

        DeclareLaunchArgument('rviz',default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,description='Configuration path of rviz2.'),

        DeclareLaunchArgument('gps',default_value='true', description=''),

        DeclareLaunchArgument('rate',default_value='15', description=''),
        DeclareLaunchArgument('queue_size',default_value='2', description=''),
        DeclareLaunchArgument('rgb2grey',default_value='false', description=''),
        DeclareLaunchArgument('mode',default_value='depth', description=''),        # 'depth' or 'disparity'

        GroupAction(
            [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_link',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom','base_footprint'],
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
                    # -90[degre]
                    # x y z yaw pitch roll
                    #arguments=['0', '0', '0.193', '-1.5707963267948966', '0', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    # -0.7[degre] 下向き の補正
                    #arguments=['0.038', '0', '0.193', '-1.5707963267948966', '-0.012217304763960306', '-1.5707963267948966', 'base_link', 'stereo_camera'],
                    #arguments=['0.038', '0', '0.193', '0', '-0.012217304763960306', '0', 'base_link', 'stereo_camera'],
                    arguments=['0.038', '0', '0.193', '0', '-0.012217304763960306', '0', 'base_link', 'oak'],
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

                Node(
                    package='hello',
                    executable='odom',
                    name='odom',
                    output="screen",
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(depthai_ros_my , 'launch', 'oak-d_stereo.launch.py')
                    ),
                    launch_arguments={'rate': LaunchConfiguration('rate'),
                                      'queue_size': LaunchConfiguration('queue_size'),
                                      'mode': LaunchConfiguration('mode'),
                                      }.items(),
                    # publish
                    #  left/camera_info
                    #  left/image_rect
                    #  left/image_rect/compressed
                    #  left/image_rect/compressedDepth
                    #  left/image_rect/theora
                    #  ------
                    #  right/camera_info
                    #  right/image_rect
                    #  right/image_rect/compressed
                    #  right/image_rect/compressedDepth
                    #  right/image_rect/theora
                    #  -----
                    #  stereo/camera_info
                    #  stereo/depth
                    #  stereo/depth/compressed
                    #  stereo/depth/compressedDepth
                    #  stereo/depth/theora
                    condition=IfCondition(LaunchConfiguration('STEREO_TEST')),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(depthai_ros_my , 'launch', 'oak-d_rgb_stereo_node.launch.py')
                    ),
                    launch_arguments={'rate': LaunchConfiguration('rate'),
                                      'queue_size': LaunchConfiguration('queue_size'),
                                      'rgb2grey': LaunchConfiguration('rgb2grey'),
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
                    condition=IfCondition(LaunchConfiguration('DEPTH_TEST')),
                ),


                Node(
                    # https://github.com/ros-perception/image_pipeline/tree/foxy/depth_image_proc/src
                    # camera_info (sensor_msgs/CameraInfo) 
                    # image_rect (sensor_msgs/Image) 
                    package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                    parameters=[{
                        "decimation": 4,
                        #"voxel_size": 0.0,
                        "voxel_size": 0.05,
                        "approx_sync": True,
                        #"exact_sync": True,
                        #"approx_sync_max_interval": 0.1 ,
                        #"approx_sync_max_interval": 0.2 ,
                        "approx_sync_max_interval": 0.5 ,
                        "qos": 1,
                    }],
                    remappings=[
                        #('disparity/image', '/disparity'),   #
                        #('disparity/camera_info', '/right/camera_info'),
                        ('depth/camera_info','/stereo/camera_info'),
                        ('depth/image','/stereo/depth'),
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
                    parameters=[rtabmap_parameters,
                        {'subscribe_stereo': True,
                         'subscribe_desp': False}],
                    remappings=rtabmap_remappings_stereo,
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
                    remappings=rtabmap_remappings_stereo,
                    namespace=LaunchConfiguration('namespace'),
                    ),

            ],
            condition=IfCondition(LaunchConfiguration('PC_STEREO_TEST')),
            #condition=IfCondition(PythonExpression(["'",LaunchConfiguration('PC'), "' == 'true'"])),
        ),

        GroupAction(
            [
                # SLAM mode:
                Node(
                    condition=UnlessCondition(localization),
                    package='rtabmap_slam', executable='rtabmap', output='screen',
                    parameters=[rtabmap_parameters,
                        {'subscribe_stereo': False,
                        'subscribe_desp': True}],
                    remappings=rtabmap_remappings_desp,
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
                    remappings=rtabmap_remappings_desp,
                    namespace=LaunchConfiguration('namespace'),
                    ),

            ],
            condition=IfCondition(LaunchConfiguration('PC_DEPTH_TEST')),
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
