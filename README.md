## Launching UR10e Gazebo + MoveIt sim
1.   roslaunch ur_e_gazebo ur10e.launch limited:=true
2.   roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch sim:=true limited:=true
3.   export LC_NUMERIC="en_US.UTF-8"
3.1. roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true

## Setting Camera Calibrator
rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.019 image:=/kinect/rgb/image_color camera:=/kinect/rgb

## Creating Target
rosrun target_finder target_gen _target_rows:=7 _target_cols:=9 _target_spacing:=0.019 _target_circle_dia:=0.0 _target_type:=0 _target_name:=Checkerboard _target_frame:=target_frame _target_file_path:=/home/fabio/Downloads/taget.yaml _transform_interface:=ros_lti

## Set Robot Speed
rosservice call /ur_hardware_interface/set_speed_slider "speed_slider_fraction: 0.1"

## Hand eye calibration steps
1. roslaunch cork_iris handeye_calibrate.launch
1.1. Run multiple movements and take samples on each of them
2. Hit the "Compute" button
3. Close the calibrate node and launch the publish one. roslaunch cork_iris handeye_publish.launch

