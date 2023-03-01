# -*- coding: utf-8 -*-
#
# rtabmap_ros_my/launch/navsat_transform.launch.py
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
# 4.1 run on SBC (Jetson Nano 2G)
#  1) term1
#   $ sudo chmod 777 /dev/ttyTHS1
#   $ sudo chmod 777 /dev/ttyUSB0
#   $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS1 -b 1000000 [-v6]
#
#  2) term2 
#   $ ros2 launch rtabmap_ros_my navsat_transform.launch.py

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


def generate_launch_description():

    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_ros'), 'launch', 'config', 'rgbd.rviz'
    )

    uvc_camera = get_package_share_directory('uvc_camera')
    stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')
    gysfdmaxb_gps=get_package_share_directory('gysfdmaxb_gps')

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
                os.path.join(gysfdmaxb_gps,'launch', 'gysfdmaxb_gps.launch.py')
            ),
            launch_arguments={
                'rate':'6',
                'device':'/dev/ttyUSB0',
                'topicName':'/gps/fix'}.items(),
            condition=IfCondition(LaunchConfiguration('gps')),
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
            #parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'navsat_transform.yaml')],
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
            #namespace=LaunchConfiguration('namespace'),
        ),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[os.path.join(get_package_share_directory("rtabmap_ros_my"), 'params','foxbot_core3', 'ekf.yaml')],
            remappings=[
                # subscribe
                ('wheel', '/odom_fox'), 
                ('odom_gps', '/odom_gps'),
                ('imu0', '/imu'),
                #('imu/data', '/imu_fox'),
                # publish
                ('odometry/filtered', '/odom'),
                ],
        ),

    ])
