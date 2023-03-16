# -*- coding: utf-8 -*-
# rtabmap_ros_my/launch/stereo_camera_test0.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# 2. run
#      SBC /dev/video0
#      PC  /dev/video1
# $ sudo chmod 777 /dev/video0
# $ ros2 launch rtabmap_ros_my stereo_camera_test0.launch.py
#
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
            launch_arguments={'left/device': '/dev/video0','qos': '1', 'intra':'False', 'trace':'True', 'fps': '12' }.items(),
            #launch_arguments={'left/device': '/dev/video0','qos': '1', 'intra':'False', 'trace':'True' }.items(),
            #launch_arguments={'left/device': '/dev/video1','qos': '1', 'intra':'False', 'trace':'True' }.items(),
        ),

        IncludeLaunchDescription(
            # http://wiki.ros.org/stereo_image_proc?distro=noetic
            PythonLaunchDescriptionSource(
                os.path.join(stereo_image_proc,'launch', 'stereo_image_proc.launch.py')
            ),
            launch_arguments={'left_namespace':'left' ,'right_namespace':'right'}.items(),
        ),

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
