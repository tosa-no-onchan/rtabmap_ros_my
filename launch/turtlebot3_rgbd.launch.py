#
# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
#
# original file : rtabmap_ros/rtabmap_demos/launch/turtlebot3_rgbd.launch.py
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_gazebo
#
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# Example:
#  1. Gazebo
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
#   how to kill Gazeb server
#   $ killall gzserver
#
#  2. rtabmap_ros with rgbd
#   SLAM:
#   $ ros2 launch rtabmap_ros_my turtlebot3_rgbd.launch.py [rviz:=True]
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint odom_topic:=/odom args:="-d" use_sim_time:=true rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info approx_sync:=true qos:=2
#   $ ros2 run topic_tools relay /rtabmap/map /map
#
#  3.Navigation (install nav2_bringup package):
#   3.1    default local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/nav2_params.yaml
#   3.2    teb_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/teb_nav2_params.yaml
#   3.3    rpp_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/rpp_nav2_params.yaml
#
#  4. rviz
#   $ ros2 launch nav2_bringup rviz_launch.py
#    下記もOK?
#   $ rviz2 -d /home/nishi/colcon_ws/src/rtabmap_ros_my/launch/config/rgbd.rviz or fox-nav2.rviz
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  5. C++ Program controll
#   #$ ros2 run turtlebot3_navi_my multi_goals4_nav2
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=True
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True
#
# append.
# how to map save
# ros2 run nav2_map_server map_saver_cli -f ~/map/my_map --ros-args -p save_map_timeout:=10000.0
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    cloud_xyzrgb = LaunchConfiguration('cloud_xyzrgb')

    # how to check rtabmap_ros params
    # rtabmap --params | grep RGBD/OptimizeMaxError
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          #'Vis/MaxDepth':'3.0',        # add by nishi 2024.3.11 No Need
          #'Vis/MeanInliersDistance':'3.0',  # add by nishi 2024.3.11 No Need
          #'Kp/MaxDepth':'3.0',  # add by nishi 2024.3.11 No Need
          #'Grid/MinGroundHeight':'0.01',   # add by nishi 2024.3.12 No Need
          'Grid/MaxGroundHeight':'0.05',    # add by nishi 2024.3.12 Very Good!! 3[M] 先の床が障害物になるのを防ぐ
          'Grid/MaxObstacleHeight':'0.7',   # add by nishi 2024.3.11 Needed
          #'Grid/RangeMax':'2.8', # add by nishi 2024.3.11 Ok !!
          #'RGBD/OptimizeMaxError' : '3.3',  # test by nishi 2024.3.12
          'RGBD/OptimizeMaxError' : '2.8',  # test by nishi 2024.3.12
    }

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
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        DeclareLaunchArgument('rviz',default_value='false', description='Launch RVIZ (optional).'),
        #DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, description='Configuration path of rviz2.'),

        DeclareLaunchArgument('cloud_xyzrgb',default_value='true', description='cloud_xyzrgb.'),

        # Nodes to launch
        Node(
            condition=IfCondition(cloud_xyzrgb),
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            parameters=[{
                "decimation": 4,
                "voxel_size": 0.0,
                "approx_sync": True,
                "approx_sync_max_interval": 0.5,
                #'qos': qos,
                'qos': 0,
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
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])
