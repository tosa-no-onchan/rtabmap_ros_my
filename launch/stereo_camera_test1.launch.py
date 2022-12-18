# -*- coding: utf-8 -*-
# rtabmap_ros_my/launch/stereo_camera_test1.launch.py
#
# ros2 launch rtabmap_ros_my stereo_camera_test1.launch.py
import os

#from launch import LaunchDescription
from launch_ros.actions import Node



from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    uvc_camera = get_package_share_directory('uvc_camera')
    stereo_image_proc = get_package_share_directory('stereo_image_proc')
    rtabmap_ros_my = get_package_share_directory('rtabmap_ros_my')

    parameters={
        'voxel_size':0.05,
        'decimation7':1,
        "max_depth":4
    }

    remappings=[
		  ('disparity/image','disparity'),
		  ('disparity/camera_info','right/camera_info'),
		  ('cloud','cloudXYZ')]

    return LaunchDescription([
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
            arguments=['0', '0', '0.2', '-1.5707963267948966', '0', '-1.5707963267948966', 'base_link', 'stereo_camera'],
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
            launch_arguments={'left/device': '/dev/video0'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(stereo_image_proc,'launch', 'stereo_image_proc.launch.py')
            ),
            launch_arguments={'left_namespace':'left' ,'right_namespace':'right'}.items(),
        ),


        Node(
            package='rtabmap_ros', executable='stereo_sync', output="screen",
            #condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
            parameters=[{
                "approx_sync": True,
                "approx_sync_max_interval": 0.0,
                "queue_size": 10,
                "qos": 2,
                "qos_camera_info": 2}],
            remappings=[
                ("left/image_rect", '/left/image_rect_color'),
                ("right/image_rect", '/right/image_rect_color'),
                ("left/camera_info", '/left/camera_info'),
                ("right/camera_info", '/right/camera_info'),
                ("rgbd_image", '/rgbd_image')],
            #namespace=LaunchConfiguration('namespace')
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
