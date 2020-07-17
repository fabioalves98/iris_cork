## Install project dependencies
rosdep install -iyr --from-paths src (--os ubuntu:bionic)

## Record Kinect Bag
rosbag record --duration=10 camera/rgb/image_raw camera/depth_registered/image_raw camera/depth_registered/points camera/depth_registered/camera_info /tf

## Set Robot Speed
rosservice call /ur_hardware_interface/set_speed_slider "speed_slider_fraction: 0.1"

## Setting Camera Calibrator
rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.019 image:=/kinect/rgb/image_color camera:=/kinect/rgb

## Standard launch
1. roslaunch iris_ur10e live.launch
2. roslaunch cork_iris kinect_launch.launch
# Calibration steps (Don't run these if calibration is already saved to the .ros directory)
3. roslaunch cork_iris handeye_calibrate.launch
4. rosrun    cork_iris arm_control.py actionlist default_caljob.csv
# After the calibration is done
5. Close roslaunch cork_iris handeye_calibrate.launch
6. roslaunch cork_iris handeye_publish.launch

## Alternatively, to launch robot driver + kinect + calibration
1. roslaunch cork_iris global_live.launch