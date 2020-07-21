#!/usr/bin/env python

import sys, copy, time, yaml, csv, json, shutil, os
import rospy, rospkg, rosparam, tf, dynamic_reconfigure.client
import tf2_ros, tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ast import literal_eval
from math import pi, cos, sin
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TransformStamped, Pose, PoseStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from easy_handeye.srv import TakeSample, ComputeCalibration
from std_srvs.srv import Empty, Trigger
from ur_msgs.srv import SetSpeedSliderFraction
from ur_dashboard_msgs.srv import Load, GetProgramState, GetLoadedProgram
from ArmControl import ArmControl
from Calibration import Calibration

arm = None
calibration = None

CORK_IRIS_BASE_DIR = rospkg.RosPack().get_path('cork_iris')
DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'

CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
## Fast control variable just for debugging purposes
SIM = False

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
            print("Moving to " + command)
            arm.jointGoal(positions[command])
        elif("grip" in command):
            arm.grip()
        elif("release" in command):
            arm.release()
        elif("save" in command):
            pos_name = args[1]
            arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", pos_name)
        elif("actionlist" in args[0]):
            actionlist_file = args[1]
            actions = parseActionlist(actionlist_file)
            runActionlist(actions)

        else:
            print("Usage: rosrun cork_iris arm_control.py <command> <command_params>")
            print("Available commands:")
            print("\tmove   <x> <y> <z> -> Simple cartesian movement relative to last position")
            print("\trotate <x> <y> <z> -> Simple rotation relative to last position")
            print("\tgrip               -> Grip the gripper")
            print("\trelease            -> Release the gripper")
            print("\t<position_name>    -> Joint goal to a <position_name>. Names are loaded at the start from positions.yaml")
            print("\tsave   <pos_name>  -> Save the current joint values of the arm")
            print("\actionlist <file>   -> Run movements defined in action list <file>")


    except Exception as e:
        if len(args) == 0:
            test()
        else:
            rospy.logerr("[CORK-IRIS] An error occured while parsing the arguments:")
            rospy.logerr(e)
            rospy.logerr("Check the help command for more information.")   



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
                continue

            if(sample and not SIM): calibration.take_sample()


# def save_calibration_to_basedir():
#     ''' Copies the saved calibration file in the calibration filepath to the cork irirs base yaml directory'''
#     try:
#         src = os.path.expanduser("~") + CALIBRATION_FILEPATH[1:]
#         dest = CORK_IRIS_BASE_DIR+"/yaml/easy_handeye_eye_on_base.yaml"
#         shutil.copyfile(src, dest)
#     except Exception as e:
#         rospy.logerr("[CORK-IRIS] Error while saving file to '" + dest + "'")
#         rospy.logerr(e)

# def load_calibration_file(filename):
#     ''' Loads a file from the yaml cork iris base directory with name <filename> and adds it to the default calibration dir'''
#     try:
#         src = CORK_IRIS_BASE_DIR+"/yaml/"+filename
#         dest = os.path.expanduser("~") + CALIBRATION_FILEPATH[1:]
#         shutil.copyfile(src, dest)
#     except Exception as e:
#         rospy.logerr("[CORK-IRIS] Error while loading file from '" + src + "'")
#         rospy.logerr(e)
#     rospy.loginfo("Calibration file loaded correctly!")
    

# def print_samples(samples):
#     print("Translation" + "\t" + "Rotation")
#     for sample in samples:
#         print("x: " + str(sample.translation.x)[:8] + "\t" + str(sample.rotation.x)[:8])
#         print("y: " + str(sample.translation.y)[:8] + "\t" + str(sample.rotation.y)[:8])
#         print("z: " + str(sample.translation.z)[:8] + "\t" + str(sample.rotation.z)[:8])
#         print("=========================================")

# def take_sample():
#     rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/take_sample', timeout=2.5)
#     take_sample_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/take_sample', TakeSample)
#     vals = take_sample_srv()
#     print("New sample taken: ")
#     transforms = vals.samples.camera_marker_samples.transforms
#     print_samples([transforms[len(transforms)-1]])

# def compute_calibration():
    
#     # get sample list - /easy_handeye_eye_on_base/get_sample_list
#     # chamar servico compute - /easy_handeye_eye_on_base/compute_calibration
#     rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', timeout=2.5)
#     compute_calibration_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', ComputeCalibration)
#     print("Computing calibration")
#     result = compute_calibration_srv()
#     print("Finished calibration.")
#     print(result)
#     print("Saving calibration to: " + CALIBRATION_FILEPATH + " and " + CORK_IRIS_BASE_DIR + "/yaml")
#     rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', timeout=2.5)
#     save_calibration = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', Empty)
#     save_calibration()
    
#     save_calibration_to_basedir()
    



def getTransform(src_tf, dest_tf):
    ''' Uses tf2 buffer lookup transform to return the transform from src_tf to dest_tf'''
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10.0)
    trans = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(str(src_tf), str(dest_tf), rospy.Time(), rospy.Duration(5))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("[CORK-IRIS] Error looking up transform!")
            rospy.logerr(e)
            return
    return trans


## Subscribing to the aruco result data is needed for him to publish the
## marker transform.
# def aruco_callback(data):
#     pass


def setPCLCorkParameter(params={"live" : "true", "type" : "4"}):
    ''' Updates the pcl_cork parameters using "params" values'''
    client = dynamic_reconfigure.client.Client("pcl_cork", timeout=30)
    client.update_configuration(params)

def computeCorkGrabPositions():
    ## Gets a position -0.25 meters behind the original cork piece. this should be the 
    ## grabbing position
    trans = getTransform('base_link', 'cork_piece')

    ## Position 25 centimeters back from the main cork position
    aux = PoseStamped()
    aux.header.stamp = rospy.Time.now()
    aux.header.frame_id = "base_link"
    aux.pose.position.x = -0.25
    aux.pose.position.y = 0
    aux.pose.position.z = 0
    aux.pose.orientation.x = 0
    aux.pose.orientation.y = 0
    aux.pose.orientation.z = 0
    aux.pose.orientation.w = 1

    grab_pose_1 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    aux.pose.position.x = -0.075
    grab_pose_2 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    return (grab_pose_1, grab_pose_2)


def grab_cork(cork, cork_grab_pose):
    '''Temporary function to grab a cork piece given its pose'''
    global arm


    goon = raw_input("grab cork?")
    if('n' in goon):
        rospy.signal_shutdown("emergency stop. dont grab")
        return

    arm.poseGoal([cork_grab_pose.position.x, cork_grab_pose.position.y, cork_grab_pose.position.z], 
    [cork_grab_pose.orientation.x,cork_grab_pose.orientation.y,cork_grab_pose.orientation.z,cork_grab_pose.orientation.w ])
    
    goon = raw_input("go towards cork?")
    if('n' in goon):
        rospy.signal_shutdown("emergency stop. dont grab")
        return

    time.sleep(2)

    arm.poseGoal([cork.position.x, cork.position.y, cork.position.z], 
    [cork.orientation.x, cork.orientation.y, cork.orientation.z, cork.orientation.w])
    goon = raw_input("press enter to grip")
    if('n' in goon):
        rospy.signal_shutdown("emergency stop. dont grip")
        return 
    arm.grip()

    
    goon = raw_input("pick up and go back?")
    if('n' in goon):
        rospy.signal_shutdown("emergency stop. dont go back")
        return 
    arm.poseGoal([cork_grab_pose.position.x, cork_grab_pose.position.y, cork_grab_pose.position.z], 
    [cork_grab_pose.orientation.x,cork_grab_pose.orientation.y,cork_grab_pose.orientation.z,cork_grab_pose.orientation.w ])
    
    goon = raw_input("go to out of camera?")
    if('n' in goon):
        rospy.signal_shutdown("emergency stop. dont go back")
        return 
    arm.jointGoal(positions['out_of_camera_pos'])

    rospy.signal_shutdown("grabbed cork debug stop")


    
    

def test():
    global positions, arm, test_publisher

    # arm.setSpeed(0.1)
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([p.x, p.y, p.z], [-0.471886915591, 0.0268562771098, 0.859489799629, -0.194624544454])

    # arm.jointGoal(positions['vert_pick_pos'])
    # arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", "init_calibration_pos")
    
    # listener = tf.TransformListener()
    # Confirm pcl cork is live and with clustering activated
    setPCLCorkParameter()
    while not rospy.is_shutdown():
        # listener.waitForTransform("/cork_piece", "/base_link", rospy.Time.now(), rospy.Duration(4.0))
        trans = None
        while not trans:
            trans = getTransform("camera_depth_optical_frame", "cork_piece")
        
        grab1, grab2 = computeCorkGrabPositions()
        setPCLCorkParameter({"live" : "false"})
        grab_cork(grab2.pose, grab1.pose)
        test_publisher.publish(grab1)
    rospy.spin()

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    global arm, positions, position_names, test_publisher, calibration
    position_names, positions = load_positions(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    # rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    # rospy.Subscriber("/cork_iris/cork_piece", PoseStamped, robot2cork)

    test_publisher = rospy.Publisher('cork_iris/grabbing_position', PoseStamped, queue_size=1)
    
    if SIM:
        arm = ArmControl('localhost')
    else:
        arm = ArmControl()
        calibration = Calibration(CORK_IRIS_BASE_DIR)
    

    arm.setSpeed(0.1)

    parseParams(sys.argv[1:])


if __name__ == "__main__":
    main()