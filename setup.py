#from setuptools import setup
from setuptools import setup, find_packages


# add by nishi
import os
from glob import glob

package_name = 'rtabmap_ros_my'

setup(
    name=package_name,
    version='0.0.0',
    #packages=[package_name],
    packages=[],
    #py_modules=[
    #    'rtabmap_ros_my.odom_pub_test',
    #    'rtabmap_ros_my.odom_simulator_func',
    #    'rtabmap_ros_my.tf_lib'
    #],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add by nishi
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'launch','config'), glob('launch/config/*.rviz')),
        (os.path.join('share', package_name,'params','foxbot_core3'), glob('params/foxbot_core3/*.yaml')),
        (os.path.join('share', package_name,'params','foxbot_nav2'), glob('params/foxbot_nav2/*.yaml')),
        (os.path.join('share', package_name,'params','rgbd_sync'), glob('params/rgbd_sync/*.yaml')),
        (os.path.join('share', package_name,'params','scan'), glob('params/scan/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nishi',
    maintainer_email='non@netosa.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #"odom_pub_test = rtabmap_ros_my.odom_pub_test:main",
        ],
    },

)
