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
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

move_group = None
robot = None
display_trajectory_publisher= None


def jointGoal(joints):
    
    joint_goal = move_group.get_current_joint_values()

    for i in range(0, len(joints)):
        if (joints[i]):
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
        waypoints.append(copy.deepcopy(wpose)) 
    if movement[1] != 0:
        wpose.position.x += movement[1] * -sin(direction)
        wpose.position.y += movement[1] * cos(direction)
        waypoints.append(copy.deepcopy(wpose))
    if movement[2] != 0:  
        wpose.position.z += movement[2]
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


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    global move_group, robot, display_trajectory_publisher
    robot = moveit_commander.RobotCommander()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

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
    #jointGoal([pi/4, -pi/2, pi/2, 0.5, pi/2, -pi/2])

    # Default joint goal for Calibraion with real robot
    # jointGoal([0.391, -1.553, 2.165, -0.226, 1.232, -1.654])
    jointGoal([pi, None, None, None, None, None])

    # To be tested
    #poseGoal([0.4, 0.3, 0.4])

    # X limit
    #simpleMove([-0.3, 0, 0], pi/4)
    # Y limit
    #simpleMove([0, -0.3, 0], pi/4)
    # Z limit
    #simpleMove([0, 0, 0.3], pi/4)
    # Horizontal align
    #simpleRotate([0, 0, pi/8])
    # X rotate
    #simpleRotate([-pi/6, 0, 0])

    
    # Caljob Example
    rospy.set_param('/caljob_creator/capture_scene', True)
    # Move X
    simpleMove([0.1, 0, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    
    simpleMove([-0.4, 0, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleMove([0.3, 0, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''
    # Rotate X in the center
    simpleRotate([-pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([pi/3, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([-pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    
    # Rotate Y in the center
    simpleRotate([0, -pi/6, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([0, pi/3, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([0, -pi/6, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    # Rotate Z in the center
    simpleRotate([0, 0, pi/8])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([0, 0, -pi/4])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([0, 0, pi/8])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''
    # Move Y in the center
    simpleMove([0, 0.25, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''
    # Rotate in the side
    simpleRotate([-pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([pi/3, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([-pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''
    simpleMove([0, -0.5, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''
    # Rotate in the side 
    simpleRotate([pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([-pi/3, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)

    simpleRotate([pi/6, 0, 0])
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    '''

    simpleMove([0, 0.25, 0], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    
    # Move Z
    simpleMove([0.15, 0, 0.25], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    
    simpleMove([-0.15, 0, -0.25], pi/4)
    time.sleep(.5)
    rospy.set_param('/caljob_creator/capture_scene', True)
    
    
    rospy.set_param('/caljob_creator/quit', True)

if __name__ == "__main__":
   main() 