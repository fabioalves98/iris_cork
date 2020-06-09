#!/usr/bin/env python

import sys
import copy
import time
import rospy
import tf2_ros, tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, cos, sin
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TransformStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from easy_handeye.srv import TakeSample, ComputeCalibration
from std_srvs.srv import Empty, Trigger
from ur_msgs.srv import SetSpeedSliderFraction
from ur_dashboard_msgs.srv import Load

move_group = None
robot = None
display_trajectory_publisher= None

DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'
CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
init_sim_pos = [-2.0818731710312974, -1.8206561665659304, -1.590893522490271, -0.5344753089586067, 1.377946515231355, 0.19641783200394514]
init_live_pos = [0.508063793182373, -1.2333410543254395, 1.8473342100726526, -0.37647326410327153, 1.3200550079345703, -0.05201179185976201]
out_of_camera_pos = [0.036165714263916016, -2.1434017620482386, 2.26279861131777, 0.18486802160229487, 0.8645825386047363, -0.19302159944643194]


def jointValues():
    return move_group.get_current_joint_values()

def getPose():
    return move_group.get_current_pose().pose



def load_and_play_program(program_filename):
    rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
    load_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
    print(load_program(program_filename))

    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    play_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
    print(play_program()) 


def grip():
    load_and_play_program('grip.urp')



def release():
    load_and_play_program('release.urp')



def setSpeed(speed):
    rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
    set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
    print(set_speed(speed))


## TEST THIS!
def getSpeed():
    pass


def jointGoal(joints):
    joint_goal = move_group.get_current_joint_values()

    for i in range(0, len(joints)):
        if (joints[i] != None):
            joint_goal[i] = joints[i]
    

    # joint_goal = joints

    # # The go command can be called with joint values, poses, or without any
    # # parameters if you have already set the pose or joint target for the group
    print("Sending Joint Action - ", joints)
    plan = move_group.go(joint_goal, wait=True)

    # # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


def poseGoal(coordinates, orientation):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = coordinates[0]
    pose_goal.position.y = coordinates[1]
    pose_goal.position.z = coordinates[2]

    pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(orientation[0], orientation[1], orientation[2]))

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
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.05,        # eef_step
                                    0.0)         # jump_threshold
    time.sleep(1.0) # Gives time for plan to show
    move_group.execute(plan, wait=True)


def simpleMove(movement, direction):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    joint_values = move_group.get_current_joint_values()

    if movement[0] != 0:
        wpose.position.x += movement[0] * cos(direction)
        wpose.position.y += movement[0] * sin(direction)
    if movement[1] != 0:
        wpose.position.x += movement[1] * -sin(direction)
        wpose.position.y += movement[1] * cos(direction)
    if movement[2] != 0:  
        wpose.position.z += movement[2]

    waypoints.append(copy.deepcopy(wpose))
    
    print("Sending Move Action - ", movement)
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
    
    print("Sending Rotate Action - ", rotation)
    cartesianGoal(waypoints)


def parseRotationArgs(args):

    for i in range(0, len(args)):
        args[i] = eval(args[i].replace('pi', str(pi)))
    return args



def parseParams(args):
    try:
        print (args)
        # X > 0 - Forward
        # y > 0 - Left
        # Z > 0 - Upwards
        if("move" in args[0]):
            simpleMove([float(args[1]), float(args[2]), float(args[3])], pi/4)
        
        elif("rotate" in args[0]):
            args = parseRotationArgs(args[1:4])
            simpleRotate([args[0], args[1], args[2]])
        
        elif("initial" in args[0]):
            jointGoal(init_live_pos)
        
        elif("caljob" in args[0]):
            caljob()
        else:
            print("Usage: rosrun cork_iris arm_control.py <command> <command_params>")
            print("Available commands:")
            print("\tmove   <x> <y> <z> -> Simple cartesian movement relative to last position")
            print("\trotate <x> <y> <z> -> Simple rotation relative to last position")
            print("\tinitial            -> Joint goal to the default initial position")
            print("\tcaljob             -> Calibration job.")

    except Exception as e:
        if len(args) == 0:
            test()
        else:
            print(e)
            print("Wrong arguments provided! Check the help command")


def caljob():
    init_pos = init_live_pos
    # Saved Position
    jointGoal(init_pos)
    take_sample()

    # X Rotation
    for i in range(3):
        simpleRotate([pi/9, 0, 0])
        take_sample()
    
    jointGoal(init_pos)

    for i in range(3):
        simpleRotate([-pi/9, 0, 0])
        take_sample()
    
    jointGoal(init_pos)
    
    # Y Rotation
    for i in range(3):
        simpleRotate([0, pi/9, 0])
        #simpleMove([-0.02, 0, 0], pi/4)
        take_sample()
    
    jointGoal(init_pos)

    simpleMove([-0.03, 0, -0.03], pi/4)
    take_sample()

    # Z Rotation
    for i in range(3):
        simpleRotate([0, 0, pi/9])
        take_sample()
    
    jointGoal(init_pos)
    simpleMove([-0.03, 0, -0.03], pi/4)

    for i in range(3):
        simpleRotate([0, 0, -pi/9])
        take_sample()
    
    jointGoal(init_pos)

    compute_calibration()


def print_samples(samples):
    print("Translation" + "\t" + "Rotation")
    for sample in samples:
        print("x: " + str(sample.translation.x)[:8] + "\t" + str(sample.rotation.x)[:8])
        print("y: " + str(sample.translation.y)[:8] + "\t" + str(sample.rotation.y)[:8])
        print("z: " + str(sample.translation.z)[:8] + "\t" + str(sample.rotation.z)[:8])
        print("=========================================")

def take_sample():
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/take_sample')
    take_sample_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/take_sample', TakeSample)
    vals = take_sample_srv()
    print("New sample taken: ")
    transforms = vals.samples.camera_marker_samples.transforms
    print_samples([transforms[len(transforms)-1]])

def compute_calibration():
    
    # get sample list - /easy_handeye_eye_on_base/get_sample_list
    # chamar servico compute - /easy_handeye_eye_on_base/compute_calibration
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration')
    compute_calibration_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', ComputeCalibration)
    print("Computing calibration")
    result = compute_calibration_srv()
    print("Finished calibration.")
    print(result)
    print("Saving calibration to: " + CALIBRATION_FILEPATH)
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration')
    save_calibration = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', Empty)
    save_calibration()
    


## Subscribing to the aruco result data is needed for him to publish the
## marker transform.
def aruco_callback(data):
    pass

def robot2cork(data):
    print ("\nCork Center")
    print ("X: " + str(data.x) + ", Y: " + str(data.y) + " Z: " + str(data.z))
    
    print ("\nRobot Pose")
    pos = getPose().position
    ori = getPose().orientation
    print ("X: " + str(pos.x) + ", Y: " + str(pos.y) + " Z: " + str(pos.z))
    print ("X: " + str(ori.x) + ", Y: " + str(ori.y) + " Z: " + str(ori.z) + " W: " + str(ori.w))

    print ("\nKinect Transform")

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10.0)
    trans = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', rospy.Time(0))
            #print (trans, rot)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "string"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = data
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    
    print (tf2_geometry_msgs.do_transform_pose(pose, trans))
    
    

def test():
    # We can get the name of the reference frame for this robot:
    #print("============ Planning frame: ", move_group.get_planning_frame())

    # We can also print the name of the end-effector link for this group:
    #print("============ End effector link: ", move_group.get_end_effector_link())

    # We can get a list of all the groups in the robot:
    #print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the robot:
    #print("============ Printing robot state:", robot.get_current_state())

    # To be tested
    # poseGoal([0.4, 0.3, 0.4])

    # Default joint goal for Calibraion with real robot
    # jointGoal([0.391, -1.553, 2.165, -0.226, 1.232, -1.70])
    # jointGoal([pi, None, None, None, None, None])

    # Default joint goal for Simulation
    # jointGoal([-3*pi/4, -pi/2, -pi/2, 0, pi/2, 0])
    # poseGoal([0.5, 0.5, 0.1], [0, pi/2, 0])
    # simpleMove([0.18, 0.22, -0.34], pi/4)
    # simpleRotate([0, pi/4, 0])

    setSpeed(0.1)

    print(jointValues())

    jointGoal(out_of_camera_pos)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    global move_group, robot, display_trajectory_publisher
    try:
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander("manipulator")
    except Exception as e:
        print("Couldn't load robot arm or move_group")
        sys.exit(0)
    
    rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    rospy.Subscriber("/cork_iris/cork_center", Point, robot2cork)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    
    setSpeed(0.1)
    parseParams(sys.argv[1:])

    rospy.spin()


if __name__ == "__main__":
   main() 