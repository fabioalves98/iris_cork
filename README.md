## Install project dependencies
rosdep install -iyr --from-paths src

## Launch Kinect

## Launch PCL_Cork

## Record Kinect Bag
rosbag record --duration=10 camera/rgb/image_raw camera/depth_registered/image_raw camera/depth_registered/points camera/depth_registered/camera_info /tf

## Play Kinect Bag


## Launching UR10e Gazebo Simulator
roslaunch iris_ur10e sim.launch

## Set Robot Speed
rosservice call /ur_hardware_interface/set_speed_slider "speed_slider_fraction: 0.1"

## Setting Camera Calibrator
rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.019 image:=/kinect/rgb/image_color camera:=/kinect/rgb

## Hand eye calibration steps
1. roslaunch cork_iris handeye_calibrate.launch
1.1. Run multiple movements and take samples on each of them
2. Hit the "Compute" button
3. Close the calibrate node and launch the publish one. roslaunch cork_iris handeye_publish.launch

## Standard launch
1. roslaunch iris_ur10e live.launch
2. roslaunch cork_iris kinect_launch.launch
### Calibration steps (Don't run these if calibration is already saved to the .ros directory)
3. roslaunch cork_iris handeye_calibrate.launch
4. rosrun    cork_iris arm_control.py caljob
### After the calibration is done
5. Close roslaunch cork_iris handeye_calibrate.launch
6. roslaunch cork_iris handeye_publish.launch
