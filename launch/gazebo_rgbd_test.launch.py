# -*- coding: utf-8 -*-
# Gazebo Turtlebot3 and rtabmap_ros rgbd camera test
# rtabmap_ros_my/launch/gazebo_rgbd_test.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# 2. Gazebo
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
#
# 3. run
#   $ ros2 launch rtabmap_ros_my gazebo_rgbd_test.launch.py
#
import os

#from launch import LaunchDescription
from launch_ros.actions import Node



from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    #uvc_camera = get_package_share_directory('uvc_camera')
    #stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')

    parameters={
        'voxel_size':0.05,
        'decimation7':1,
        "max_depth":4
    }

    #remappings=[
	#	  ('disparity/image','disparity'),
	#	  ('disparity/camera_info','right/camera_info'),
	#	  ('cloud','cloudXYZ')]


    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]


    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time, 'qos':qos}],
            #remappings=remappings
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('depth/image', '/camera/depth/image_raw')]
        ),

        Node(
            package='rtabmap_ros', executable='point_cloud_xyzrgb', output='screen',
            parameters=[{
                "decimation": 4,
                "voxel_size": 0.0,
                "approx_sync": True,
                "approx_sync_max_interval": 0.5,
                'qos': qos,
            }],
            remappings=[
                #('left/image', '/left/image'),
                #('right/image', '/right/image'),
                #('left/camera_info', '/left/camera_info'),
                #('right/camera_info', '/right/camera_info'),
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                #('rgbd_image', '/rgbd_image'),
                ('cloud', '/cloudXYZ')]
        ),


        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(rtabmap_ros_my, 'stereo_image_proc.launch.py')
        #    ),
        #    #launch_arguments={'left/device': '/dev/video0'}.items(),
        #),
    ])

'''
$ ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 base_link camera 3


IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(rtabmap_ros_my, 'launch', 'stereo_image_proc.launch.py')
    ),
    launch_arguments={'approximate_syn': True,
                        'use_system_default_qos': True}.items(),
),
'''
