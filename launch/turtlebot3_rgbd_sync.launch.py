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
# original file : rtabmap_ros/rtabmap_demos/ros2/turtlebot3_rgbd_sync.launch.py
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_gazebo
#
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my
#  $ . install/setup.bash
#
# Example:
#  1. Gazebo
#   $ export TURTLEBOT3_MODEL=waffle
#   $ . /usr/share/gazebo/setup.sh
#   $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
#   #$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#   how to kill Gazeb server
#   $ killall gzserver
#
#  2. rtabmap_ros with rgbd_sync
#   SLAM:
#   $ ros2 launch rtabmap_ros_my turtlebot3_rgbd_sync.launch.py [localization:=true]
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint subscribe_scan:=true  approx_sync:=true approx_rgbd_sync:=false odom_topic:=/odom args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Reg/Force3DoF true --Grid/RangeMin 0.2" use_sim_time:=true rgbd_sync:=true rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info qos:=2
#   $ ros2 run topic_tools relay /rtabmap/map /map
#
#  3. Navigation (install nav2_bringup package):
#   3.1    default local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/nav2_params.yaml
#   3.2    teb_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/teb_nav2_params.yaml
#   3.3    rpp_local_planner
#   $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/nishi/colcon_ws/src/rtabmap_ros_my/params/rgbd_sync/rpp_nav2_params.yaml
#
#  3.1 Rviz
#   $ ros2 launch nav2_bringup rviz_launch.py
#    下記もOK?
#   $ rviz2 -d /home/nishi/colcon_ws/src/rtabmap_ros_my/launch/config/rgbd.rviz
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  4. C++ Program controll
#   #$ ros2 run turtlebot3_navi_my multi_goals4_cmd_vel
#   #$ ros2 run turtlebot3_navi_my multi_goals4_nav2
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=True
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True
#
#  5. C++ Auto Map [navigation and slam]
#   $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py use_sim_time:=True
#
#  6. C++ Auto Mower
#   SLAM: localization go
#   $ ros2 launch rtabmap_ros_my turtlebot3_rgbd_sync.launch.py localization:=true
#
#   $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True
#
# append.
# how to map save
# ros2 run nav2_map_server map_saver_cli -f ~/map/house_map --ros-args -p save_map_timeout:=10000.0


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          #'subscribe_scan':False,   # changed by nhisi 2024.3.1
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          # add by nishi
          "map_always_update":True,
          "wait_for_transform_duration":"0.1",
          "Grid/Sensor":'0',
          #"Grid/RangeMax":'2.0',
          #'RGBD/AngularUpdate':"0.05",   # Update map only if the robot is moving 
          #'RGBD/LinearUpdate':"0.05",    # Update map only if the robot is moving
    }
    # tf_delay      = 0.050000
    # tf_tolerance  = 0.100000
    remappings=[
          ('rgb/image', '/camera/image_raw'),           # sensor_msgs/msg/Image
          ('rgb/camera_info', '/camera/camera_info'),   # sensor_msgs/msg/CameraInfo
          ('depth/image', '/camera/depth/image_raw')]   # sensor_msgs/msg/Image

    remappings2=[
          ('odom', '/odom'),
          ('scan', '/scan'),
          # add by nishi
          ('rgbd_image', '/rgbd_image'),
          #('rgbd_image', '/rgbd_image/compressed'),
          #('depth/image', '/camera/depth/image_raw'),
          ('map','/map')]


    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',description='Use simulation (Gazebo) clock if true'),
        #DeclareLaunchArgument('qos', default_value='2',description='QoS used for input sensor topics'),
        DeclareLaunchArgument('qos', default_value='1',description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false',description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz',default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('namespace', default_value='rtabmap', description=''),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time, 'qos':qos}],
            # http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Frgbd_sync
            #1. Subscribed Topics
            #    rgb/image (sensor_msgs/Image)
            #        RGB image stream. 
            #    depth/image (sensor_msgs/Image)
            #        Registered depth image stream. 
            #    rgb/camera_info (sensor_msgs/CameraInfo)
            #        RGB camera metadata. 
            #2. Published Topics
            #    rgbd_image (rtabmap_ros/RGBDImage)
            #        The RGB-D image topic 
            #    rgbd_image/compressed (rtabmap_ros/RGBDImage)
            #        The compressed RGB-D image topic (rgb=jpeg, depth=png) 
            remappings=remappings),

        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
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


        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            #remappings=remappings,
            remappings=remappings2,
            arguments=['-d'],
            namespace=LaunchConfiguration('namespace'),
            ),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            #remappings=remappings,
            remappings=remappings2,
            namespace=LaunchConfiguration('namespace'),
            ),

        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            #remappings=remappings,
            remappings=remappings2,
            namespace=LaunchConfiguration('namespace'),
            ),
    ])
