#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, cos, sin
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


def cartesianGoal():
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z += -0.1  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    # angle = pi/6
    # ax, ay, az = 0, 1, 0
    # wpose.orientation.x = ax * sin(angle/2)
    # wpose.orientation.y = ay * sin(angle/2)
    # wpose.orientation.z = az * sin(angle/2)
    # wpose.orientation.w = cos(angle/2)
    quat = geometry_msgs.msg.Quaternion(*quaternion_from_euler(0, pi/6, 0))
    wpose.orientation = quat
     
    waypoints.append(copy.deepcopy(wpose))

    #wpose.position.y += -0.1  # Third move sideways (y)
    #waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    move_group.execute(plan, wait=True)
    print(move_group.get_current_joint_values())


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
    print ""

    jointGoal([0, -pi/2, pi/2, 0.5, pi/2, -pi])
    # poseGoal([0.4, 0.3, 0.4])
    # cartesianGoal()


if __name__ == "__main__":
   main() 