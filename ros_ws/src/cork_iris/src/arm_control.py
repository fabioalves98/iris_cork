#!/usr/bin/env python

import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, cos, sin
from std_msgs.msg import String
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from easy_handeye.srv import *

move_group = None
robot = None
display_trajectory_publisher= None


def jointGoal(joints):
    
    joint_goal = move_group.get_current_joint_values()

    for i in range(0, len(joints)):
        if (joints[i] != None):
            joint_goal[i] = joints[i]
    

    # joint_goal = joints

    # # The go command can be called with joint values, poses, or without any
    # # parameters if you have already set the pose or joint target for the group
    plan = move_group.go(joint_goal, wait=True)

    # # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


def poseGoal(pose):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # display_trajectory(plan)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()


def cartesianGoal(waypoints):
    # waypoints = []
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.05,        # eef_step
                                    0.0)         # jump_threshold
    time.sleep(1.0) # Gives time for plan to show
    print("Executing Cartesian Plan")
    move_group.execute(plan, wait=True)



# Every movement is parallel to x,y or z. Never diagonal
def simpleMove(movement, direction):
    
    waypoints = []
    wpose = move_group.get_current_pose().pose
    joint_values = move_group.get_current_joint_values()

    if movement[0] != 0:
        wpose.position.x += movement[0] * cos(direction)
        wpose.position.y += movement[0] * sin(direction)
        #waypoints.append(copy.deepcopy(wpose)) 
    if movement[1] != 0:
        wpose.position.x += movement[1] * -sin(direction)
        wpose.position.y += movement[1] * cos(direction)
        #waypoints.append(copy.deepcopy(wpose))
    if movement[2] != 0:  
        wpose.position.z += movement[2]
        #waypoints.append(copy.deepcopy(wpose)) 

    waypoints.append(copy.deepcopy(wpose))
    print("Sending Cartesian Goal")
    cartesianGoal(waypoints)

def simpleRotate(rotation):
    
    waypoints = []
    wpose = move_group.get_current_pose().pose

    orignal = quaternion_from_euler(0, 0, 0)
    orignal[0] = wpose.orientation.x
    orignal[1] = wpose.orientation.y
    orignal[2] = wpose.orientation.z
    orignal[3] = wpose.orientation.w

    quat = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    
    wpose.orientation = geometry_msgs.msg.Quaternion(*quaternion_multiply(orignal, quat))
    waypoints.append(copy.deepcopy(wpose))
    
    print("Sending Cartesian Goal")
    cartesianGoal(waypoints)


def parseParams(args):
    
    if("move" in args[0]):
        try:
            simpleMove([args[1], args[2], args[3]], pi/4)
        except Exception as e:
            print("Wrong arguments provided! Check the help command.")
    elif("rotate" in args[0]):
        try:
            simpleRotate([args[1], args[2], args[3]])
        except Exception as e:
            print("Wrong arguments provided! Check the help command.")
    elif("help" in args[0]):
        print("help")
    # else:
        # default_caljob()


def print_samples(samples):
    print("Translation" + "\t" + "Rotation")
    for sample in samples:
        print("x: " + str(sample.translation.x)[:8] + "\t" + str(sample.rotation.x)[:8])
        print("y: " + str(sample.translation.y)[:8] + "\t" + str(sample.rotation.y)[:8])
        print("z: " + str(sample.translation.z)[:8] + "\t" + str(sample.rotation.z)[:8])
        print("=========================================")

def take_sample():
    rospy.wait_for_service('/easy_handeye_eye_on_base/take_sample')
    take_sample_srv = rospy.ServiceProxy('/easy_handeye_eye_on_base/take_sample', TakeSample)
    vals = take_sample_srv()
    print("New sample taken: ")
    transforms = vals.samples.camera_marker_samples.transforms
    print_samples([transforms[len(transforms)-1]])

def compute_calibration():
    
    # get sample list - /easy_handeye_eye_on_base/get_sample_list
    # chamar servico compute - /easy_handeye_eye_on_base/compute_calibration
    rospy.wait_for_service('/easy_handeye_eye_on_base/compute_calibration')
    compute_calibration_srv = rospy.ServiceProxy('/easy_handeye_eye_on_base/compute_calibration', ComputeCalibration)
    print("Computing calibration")
    result = compute_calibration_srv()
    print("Finished calibration")
    print(result)
    # chamar servico save - /easy_handeye_eye_on_base/save_calibration

## Subscribing to the aruco result data is needed for him to publish the
## marker transform.
def aruco_callback(data):
    pass

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    # parseParams(sys.argv[1:])
    # input("dsa")

    global move_group, robot, display_trajectory_publisher
    robot = moveit_commander.RobotCommander()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

    rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: ", planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: ", eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())

    # Default joint goal for Simulation
    jointGoal([pi/4, -pi/2, pi/2, 0.7, pi/2, pi])
    simpleMove([0.1, -0.12, -0.2], pi/4)
    
    # Default joint goal for Calibraion with real robot
    # jointGoal([0.391, -1.553, 2.165, -0.226, 1.232, -1.70])
    # jointGoal([pi, None, None, None, None, None])

    # To be tested
    # poseGoal([0.4, 0.3, 0.4])

    # Caljob Example
    # Move X
    simpleMove([-0.05, 0, 0], pi/4)
    take_sample()
    # time.sleep(.5)
    simpleMove([0, 0, 0.1], pi/4)
    take_sample()
    simpleRotate([0, -pi/6, 0])
    take_sample()
    compute_calibration()
    

if __name__ == "__main__":
   main() 