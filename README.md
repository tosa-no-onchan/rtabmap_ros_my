## rtabmap_ros_my for Ros2 galactic
    
ROS2 対応 自作 Turtlebot3(foxbot_core3_r2.ino) で、 Single USB Stereo Camera を使って    
rtabmap_ros が実行できる、 launch ファイル。    
    
##### 1. インストール方法    

    $ cd ~colcon_ws/src    
    $ git clone https://github.com/tosa-no-onchan/rtabmap_ros_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select rtabmap_ros_my    
    
#### Update.    
    
    2022.12.18    
        support ros2 foxy    
    2023.3.1  
        support ros2 galactic  
