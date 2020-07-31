#!/usr/bin/env python

import sys, time, csv #os #yaml
import rospy, rospkg, rosparam, tf #dynamic_reconfigure.client
# import tf2_ros, tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ast import literal_eval
from math import pi, cos, sin

from cork_iris.msg import ArmCommand
from cork_iris.srv import ControlArm
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TransformStamped, Pose, PoseStamped, Quaternion
# from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
# from easy_handeye.srv import TakeSample, ComputeCalibration
# from std_srvs.srv import Empty, Trigger
# from ur_msgs.srv import SetSpeedSliderFraction
# from ur_dashboard_msgs.srv import Load, GetProgramState, GetLoadedProgram
from ArmControl import ArmControl
from Calibration import Calibration
from HelperFunctions import *

arm = None
calibration = None
scene = None

CORK_IRIS_BASE_DIR = rospkg.RosPack().get_path('cork_iris')
DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'

CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
## Fast control variable just for debugging purposes
SIM = True

test_publisher = None

positions = {}
position_names = []

def load_positions(path):
    '''Returns tuple with keys of all possible positions and the dict with the positions previously
       saved in a yaml file'''

    data = rosparam.load_file(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    k = data[0][0].keys()

    return k, data[0][0]

def parseRotationArgs(args):

    for i in range(0, len(args)):
        args[i] = eval(args[i].replace('pi', str(pi)))
    return args

def helpString():
    help = "Usage: rosrun cork_iris arm_controller.py <command> <command_params>\n \
    Available commands:\n \
    \tmove   <x> <y> <z> -> Simple cartesian movement relative to last position\n \
    \trotate <x> <y> <z> -> Simple rotation relative to last position\n \
    \tgrip               -> Close the gripper fingers\n \
    \trelease            -> Release the gripper\n \
    \t<position_name>    -> Joint goal to a <position_name>. Names are loaded at the start from positions.yaml\n \
    \tsave   <pos_name>  -> Save the current joint values of the arm\n \
    \tactionlist <file>  -> Run movements defined in action list <file>\n \
    \tgrab_cork          -> If everything is setup correctly, the controller should send signal to grab a cork piece\n"
    # print(help)
    return help

# def takeCommand(data):
#     parseParams(data.command)

def takeCommandService(req):
    output = parseParams(req.command)
    if output == -1:
        return [False, 'An error occured!']
    return [True, output]


def parseParams(args):
    global positions, position_names
    print("PARSING ARGS")
    try:
        print (args)
        command = args[0]
        # X > 0 - Forward
        # y > 0 - Left
        # Z > 0 - Upwards
        if("move" in command):
            arm.simpleMove([float(args[1]), float(args[2]), float(args[3])], pi/4)
        elif("rotate" in command):
            args = parseRotationArgs(args[1:4])
            arm.simpleRotate([args[0], args[1], args[2]])
        elif(command in position_names):
            arm.jointGoal(positions[command])
            return 'Moving to' + command
        elif("grip" in command):
            arm.grip()
        elif("release" in command):
            arm.release()
        elif("save" in command):
            pos_name = args[1]
            arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", pos_name)
            position_names, positions = load_positions(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
        elif("actionlist" in args[0]):
            actionlist_file = args[1]
            actions = parseActionlist(actionlist_file)
            runActionlist(actions)

        else:
            return helpString()

    except Exception as e:
        if len(args) == 0:
            test()
        else:
            rospy.logerr("[CORK-IRIS] An error occured while parsing the arguments:")
            rospy.logerr(e)
            rospy.logerr("Check the help command for more information.")   
            return -1


def parseActionlist(filename):
    try:
        action_file = open(filename, 'r')
    except Exception as e:
        try:
            action_file = open(CORK_IRIS_BASE_DIR + "/actionlists/" + filename)
        except OSError:
            sys.exit(0)
    
    csv_reader = csv.reader(action_file, delimiter=',')
    actions = []
    line_nr = 0
    for row in csv_reader:
        if(line_nr != 0):
            if("[" in row[2]): # If the arguments column is an array of x, y, z information
                # transform it into a python array of x,y,z information and use the 
                # parseRotationArgs to transform string values (pi/2 -> 1.57 etc) into real numbers
                arguments = parseRotationArgs(row[2].replace("[", "").replace("]", "").split(" "))
            else: arguments = row[2]
            actions.append((row[0], row[1], arguments, row[3]))
        line_nr += 1

    return actions


def runActionlist(actions):
    global arm, calibration
    for action in actions:
        reps, cmd, argums, sample = action

        for i in range(0, int(reps)):
            if "jointGoal" in cmd:
                arm.jointGoal(positions[argums])
            elif "rotate" in cmd:
                arm.simpleRotate(argums)
            elif "move" in cmd:
                arm.simpleMove(argums, pi/4)
            elif "compute" in cmd and not SIM:
                calibration.compute_calibration()
                break

            if(sample and not SIM): calibration.take_sample()



def computeCorkGrabPositions():
    ## Gets a position -0.15 meters behind the original cork piece. this should be the 
    ## grabbing position
    trans = getTransform('base_link', 'cork_piece')

    aux = newPoseStamped([-0.15, 0, 0], frame_id="base_link")
    grab_pose_1 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    if grab_pose_1.pose.position.z < trans.transform.translation.z:

        rospy.loginfo("Inverting cork piece orientation!")
        inv_quaternion = tf.transformations.quaternion_multiply(
            (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w),
            quaternion_from_euler(0, pi, 0))

        trans.transform.rotation = Quaternion(*inv_quaternion)
        aux.pose.position.x = -0.15

        grab_pose_1 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    aux.pose.position.x = -0.070
    grab_pose_2 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    ## TODO: fix this bug
    if not SIM:
        grab_pose_1.pose.position.x = grab_pose_1.pose.position.x - 0.02
        grab_pose_2.pose.position.x = grab_pose_2.pose.position.x - 0.02
    
    return (grab_pose_1, grab_pose_2)


def grab_cork(cork, cork_grab_pose):
    '''Temporary function to grab a cork piece given its pose'''
    global arm, scene


    if(not keep_going("grab_cork")):
        return

    arm.poseGoal(posePositionToArray(cork_grab_pose.position), poseOrientationToArray(cork_grab_pose.orientation))
   
    if(not keep_going("go towards cork")):
        return 

    time.sleep(2)
    
    arm.poseGoal(posePositionToArray(cork.position), poseOrientationToArray(cork.orientation))

    if(not keep_going("grip")):
        return 
    
    # TODO: Change the gripper function to take care of attaching stuff or not (and release function to detach)
    # TODO: More testing in live
    # scene.attach_box("ee_link", "cork_piece", touch_links=arm.getLinks())
    # time.sleep(2)
    # print(scene.get_attached_objects())
    arm.grip()

    
    if(not keep_going("stand back")):
        return 


    # TODO: Move towards the center of the box in an upwards direction, not always upwards like this here
    arm.simpleMove([0, 0, 0.15])
    if(not keep_going("out of camera")):
        return 
    arm.jointGoal(positions['out_of_camera_pos'])

    if(not keep_going("place pos")):
        return 
    arm.jointGoal(positions['place_pos'])

    if(not keep_going("release")):
        return
    arm.release()

    if(not keep_going("out of camera")):
        return 
    arm.jointGoal(positions['out_of_camera_pos'])


    # rospy.signal_shutdown("grabbed cork debug stop")


def grab_cork_routine():
    rospy.loginfo("Grabbing routine started")

    # Confirm pcl cork is live and with clustering activated
    setPCLCorkParameter()
    while not rospy.is_shutdown():
        # listener.waitForTransform("/cork_piece", "/base_link", rospy.Time.now(), rospy.Duration(4.0))
        trans = None
        while not trans:
            trans = getTransform("camera_depth_optical_frame", "cork_piece")
        
        grab1, grab2 = computeCorkGrabPositions()
        setPCLCorkParameter({"live" : "false"})
        test_publisher.publish(grab1)
        
        grab_cork(grab2.pose, grab1.pose)
        rospy.loginfo("Ended grabbing routine")
        break
        



def test():
    global positions, arm, test_publisher
    rospy.logwarn("arm_controller.py was called without parameters")
    # arm.setSpeed(0.1)
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([p.x, p.y, p.z], [-0.471886915591, 0.0268562771098, 0.859489799629, -0.194624544454])

    # arm.jointGoal(positions['vert_pick_pos'])
    # arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", "init_calibration_pos")
    
  
    
    # rospy.spin()

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=False)

    global arm, positions, position_names, test_publisher, calibration, scene

    position_names, positions = load_positions(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    # rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    # rospy.Subscriber("/cork_iris/cork_piece", PoseStamped, robot2cork)
    # rospy.Subscriber("/cork_iris/control", ArmCommand, takeCommand)
    cmd_receiver = rospy.Service('/cork_iris/control_arm', ControlArm, takeCommandService)

    test_publisher = rospy.Publisher('cork_iris/grabbing_position', PoseStamped, queue_size=1)
    
    if SIM:
        rospy.logwarn("[CORK-IRIS] Connecting to simulation. Change the arm_control code var to change this.")
        arm = ArmControl('localhost')
    else:
        arm = ArmControl()
        calibration = Calibration(CORK_IRIS_BASE_DIR)
        arm.setSpeed(0.1)
        arm.config_gripper(100.0)
    
    # scene = moveit_commander.PlanningSceneInterface()
    
    # time.sleep(2)
    # print(scene.get_attached_objects())
    rospy.spin()
    # parseParams(sys.argv[1:])


if __name__ == "__main__":
    main()