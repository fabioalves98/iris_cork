## Install project dependencies
rosdep install -iyr --from-paths src (--os ubuntu:bionic)

## Record Kinect Bag
rosbag record --duration=10 camera/rgb/image_raw camera/depth_registered/image_raw camera/depth_registered/points camera/depth_registered/camera_info /tf

## Set Robot Speed
rosservice call /ur_hardware_interface/set_speed_slider "speed_slider_fraction: 0.1"

## Setting Camera Calibrator
rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.019 image:=/kinect/rgb/image_color camera:=/kinect/rgb

## Standard launch
1. roslaunch cork_iris global_live.launch (with calibrate and box params)
# Arm Controller
1. rosrun    cork_iris arm_controller.py (with positions file argument)
# Calibration
1. rosrun    cork_iris arm_cmd_sender.py actionlist (with caljob file argument)
# PCL Cork
1. roslaunch cork_iris pcl_cork.launch