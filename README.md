## rtabmap_ros_my for Ros2 humble
    
ROS2 対応 自作 Turtlebot3(foxbot_core3_r2.ino) で、 Single USB Stereo Camera, OAK-D Lite を使って    
rtabmap_ros が実行できる、 launch ファイル。    
    
##### 1. インストール方法    

    $ cd ~colcon_ws/src    
    $ git clone https://github.com/tosa-no-onchan/rtabmap_ros_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my    
    
#### 2. Run Gazebo and Turtlebot3 with Rtabmap_ros Rgbd Camera    

    turtlebot3_scan.launch.py  
    turtlebot3_rgbd.launch.py  
    turtlebot3_rgbd_sync.launch.py  

turtlebot3_rgbd.launch.py または、turtlebot3_rgbd_sync.launch.py の中の記載に従って  
$ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True  
を実行すれば、  
Gazebo House and Turtlebot3 で、部屋の中を自動で動き回って、House の Mapping をします。  

注)turtlebot3_rgbd_sync.launch.py の rtabmap, point_cloud_xyzrgb の qos=1 に修正しました。by nishi 2024.9.16  
これで、turtlebot3_rgbd_sync.launch.py でも、 C++ Auto Map が問題なく動くと思います。  

[turtlebot3_navi_my](https://github.com/tosa-no-onchan/turtlebot3_navi_my)  
    
#### 3. Run real Robot(foxbot_core3_r2)    
    
    rtabmap_stereo_rgbd.launch.py  
    rtabmap_stereo_rgbd_gps.launch.py  

#### 3.1 Run real Robot(foxbot_core3_r2) with OAK-D Lite and Gps   
Active SLAM and Mapping with rtabmap_ros  
    
    rtabmap_oak-d_rgb_depth_gps.launch.py  
    rtabmap_oak-d_stereo_gps.launch.py 

#### 3.2 Run real Robot(foxbot_core3_r2) with Gps, Navigation only   
ekf robot_localization and navigation2  
A prebuild static Map is necessary.
    
    foxbot_nav2_oak-d_depth_gps.launch.py  
    foxbot_nav2_stereo_gps.launch.py  

#### 4. Update.    
2023.12.18  
    
    support ros2 foxy    
    
2023.3.1  
    
    support ros2 galactic  

2023.3.16  

    support heart beat for foxbot_core3_r2 

2024.2.8  

    support ros2 humble and updated rtabmap and rtabmap_ros  
    turtlebot3_rgbd.launch.py  
    turtlebot3_rgbd_sync.launch.py

2024.9.16  

    append launch  
    foxbot_nav2_oak-d_depth_gps.launch.py  
    foxbot_nav2_stereo_gps.launch.py  
    rtabmap_oak-d_rgb_depth_gps.launch.py  
    rtabmap_oak-d_stereo_gps.launch.py  

