#!/usr/bin/env python

import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

move_group = None
robot = None
display_trajectory_publisher= None


def jointGoal(joints):
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal = joints

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
    move_group.execute(plan, wait=True)


# direction should be an array like: [0, 0, 1]
# where the components that need to be moved should be 1
# Every movement is parallel to x,y or z. Never diagonal
def simpleMove(direction, distance):
    
    waypoints = []
    wpose = move_group.get_current_pose().pose
    
    if direction[0] == 1:
        wpose.position.x += distance[0] * direction[0] 
        waypoints.append(copy.deepcopy(wpose)) 
    if direction[1] == 1:
        wpose.position.y += distance[1] * direction[1] 
        waypoints.append(copy.deepcopy(wpose))
    if direction[2] == 1:  
        wpose.position.z += distance[2] * direction[2]
        waypoints.append(copy.deepcopy(wpose)) 

    cartesianGoal(waypoints)

def simpleRotate(axis, angle):
    
    waypoints = []
    wpose = move_group.get_current_pose().pose

    quat = geometry_msgs.msg.Quaternion(*quaternion_from_euler(axis[0] * angle[0], axis[1] * angle[1], axis[2] * angle[2]))
    wpose.orientation = quat
    waypoints.append(copy.deepcopy(wpose))
    
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
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()


    jointGoal([0, -pi/2, pi/2, 0.5, pi/2, -pi/2])
    # poseGoal([0.4, 0.3, 0.4])

    simpleMove([0, 1, 0], [0.1, 0.1, 0])
    rospy.set_param('/caljob_creator/capture_scene', True)
    time.sleep(2.0)
    simpleRotate([0, 1, 0], [0, pi/3, 0])
    rospy.set_param('/caljob_creator/capture_scene', True)
    time.sleep(2.0)
    simpleRotate([1, 0, 0], [pi, 0, 0])
    rospy.set_param('/caljob_creator/capture_scene', True)
    time.sleep(2.0)
    simpleMove([0, 1, 0], [0.1, -0.1, 0])
    rospy.set_param('/caljob_creator/quit', True)



if __name__ == "__main__":
   main() 