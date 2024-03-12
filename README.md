## rtabmap_ros_my for Ros2 humble
    
ROS2 対応 自作 Turtlebot3(foxbot_core3_r2.ino) で、 Single USB Stereo Camera を使って    
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
    
#### 3. Run real Robot(foxbot_core3_r2)    
    
    rtabmap_stereo_rgbd.launch.py  
    rtabmap_stereo_rgbd_gps.launch.py  

#### 4. Update.    
2033.12.18  
    
    support ros2 foxy    
    
2023.3.1  
    
    support ros2 galactic  

2023.3.16  

    support heart beat for foxbot_core3_r2 

2024.2.8  

    support ros2 humble and updated rtabmap and rtabmap_ros  
    turtlebot3_rgbd.launch.py  
    turtlebot3_rgbd_sync.launch.py
    
