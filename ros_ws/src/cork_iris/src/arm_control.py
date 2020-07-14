#!/usr/bin/env python

import sys, copy, time, yaml, csv, json
import rospy, rospkg, rosparam
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
from ur_dashboard_msgs.srv import Load
from ArmControl import ArmControl

arm = None

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
        elif("caljob" in command):
            caljob()
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
            print("\tinitial            -> Joint goal to the default initial position")
            print("\tcaljob             -> Calibration job")
            print("\tsave   <pos_name>  -> Save the current joint values of the arm")
            print("\actionlist <file>   -> Run movements defined in action list <file>")


    except Exception as e:
        if len(args) == 0:
            test()
        else:
            print("An error occured while parsing the arguments:")
            print(e)
            print("Check the help command for more information.")   



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
    global arm
    for action in actions:
        reps, cmd, argums, sample = action
        print(action)
        print(int(reps))
        for i in range(0, int(reps)):
            if "jointGoal" in cmd:
                arm.jointGoal(positions[argums])
            elif "rotate" in cmd:
                arm.simpleRotate(argums)
            elif "move" in cmd:
                arm.simpleMove(argums, pi/4)
            elif "compute" in cmd and not SIM:
                compute_calibration()

            if(sample and not SIM): take_sample()
        

def caljob():
    global positions
    init_pos = positions['init_calibration_pos']
    # Saved Position
    arm.jointGoal(init_pos)
    take_sample()

    # X Rotation
    for i in range(3):
        arm.simpleRotate([pi/9, 0, 0])
        take_sample()
    
    arm.jointGoal(init_pos)

    for i in range(3):
        arm.simpleRotate([-pi/9, 0, 0])
        take_sample()
    
    arm.jointGoal(init_pos)
    
    # Y Rotation
    for i in range(3):
        arm.simpleRotate([0, pi/9, 0])
        #simpleMove([-0.02, 0, 0], pi/4)
        take_sample()
    
    arm.jointGoal(init_pos)

    arm.simpleMove([-0.03, 0, -0.03], pi/4)
    take_sample()

    # Z Rotation
    for i in range(3):
        arm.simpleRotate([0, 0, pi/9])
        # take_sample()
    
    arm.jointGoal(init_pos)
    # arm.simpleMove([-0.03, 0, -0.03], pi/4)

    for i in range(3):
        arm.simpleRotate([0, 0, -pi/9])
        # take_sample()
    
    arm.jointGoal(init_pos)

    compute_calibration()


def print_samples(samples):
    print("Translation" + "\t" + "Rotation")
    for sample in samples:
        print("x: " + str(sample.translation.x)[:8] + "\t" + str(sample.rotation.x)[:8])
        print("y: " + str(sample.translation.y)[:8] + "\t" + str(sample.rotation.y)[:8])
        print("z: " + str(sample.translation.z)[:8] + "\t" + str(sample.rotation.z)[:8])
        print("=========================================")

def take_sample():
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/take_sample', timeout=2.5)
    take_sample_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/take_sample', TakeSample)
    vals = take_sample_srv()
    print("New sample taken: ")
    transforms = vals.samples.camera_marker_samples.transforms
    print_samples([transforms[len(transforms)-1]])

def compute_calibration():
    
    # get sample list - /easy_handeye_eye_on_base/get_sample_list
    # chamar servico compute - /easy_handeye_eye_on_base/compute_calibration
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', timeout=2.5)
    compute_calibration_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', ComputeCalibration)
    print("Computing calibration")
    result = compute_calibration_srv()
    print("Finished calibration.")
    print(result)
    print("Saving calibration to: " + CALIBRATION_FILEPATH)
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', timeout=2.5)
    save_calibration = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', Empty)
    save_calibration()
    


## Subscribing to the aruco result data is needed for him to publish the
## marker transform.
def aruco_callback(data):
    pass

def robot2cork(data):
    global test_publisher
    corkpose = data.pose
    print ("\nCork Pose")
    print ("X: " + str(corkpose.position.x) + ", Y: " + str(corkpose.position.y) + " Z: " + str(corkpose.position.z))
    print ("X: " + str(corkpose.orientation.x) + ", Y: " + str(corkpose.orientation.y) + " Z: " + str(corkpose.orientation.z) + " W: " + str(corkpose.orientation.w))

    
    print ("\nRobot Pose")
    pos = arm.getPose().position
    ori = arm.getPose().orientation
    print ("X: " + str(pos.x) + ", Y: " + str(pos.y) + " Z: " + str(pos.z))
    print ("X: " + str(ori.x) + ", Y: " + str(ori.y) + " Z: " + str(ori.z) + " W: " + str(ori.w))

    print ("\nKinect Transform")

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10.0)
    trans = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', rospy.Time(), rospy.Duration(5))
            # print (trans, rot)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Error looking up transform!")
            print(e)
            return
    
    # pose = geometry_msgs.msg.PoseStamped()
    # pose.header.frame_id = "string"
    # pose.header.stamp = rospy.Time.now()
    # pose.pose = data
    print("\nTransformed cork pose")
    transformed_pose = tf2_geometry_msgs.do_transform_pose(data, trans)
    print(transformed_pose.pose)

    test = copy.deepcopy(transformed_pose.pose)
    o = test.orientation


    x = 1 - 2 * (o.y * o.y + o.z * o.z)
    y = 2 * (o.x * o.y + o.w * o.z)
    z = 2 * (o.x * o.z - o.w * o.y)

    test.position.x -= x*0.15
    test.position.y -= y*0.15
    test.position.z -= z*0.15

    to_pub = PoseStamped()
    to_pub.header.stamp = rospy.Time.now()
    to_pub.header.frame_id = "base_link"
    to_pub.pose = test
    test_publisher.publish(to_pub)

    # print("GRABBING CORK")

    
    grab_cork(test)




def grab_cork(cork_pose):
    '''Temporary function to grab a cork piece given its pose'''
    global arm
    # arm.jointGoal(positions['vert_pick_pos'])
    # time.sleep(2)
    ## Orient the arm
    arm.poseGoal([cork_pose.position.x, cork_pose.position.y, cork_pose.position.z], [cork_pose.orientation.x,cork_pose.orientation.y,cork_pose.orientation.z,cork_pose.orientation.w ])
    
    
    # o = arm.getPose().orientation

    # x = 1 - 2 * (o.y * o.y + o.z * o.z)
    # y = 2 * (o.x * o.y + o.w * o.z)
    # z = 2 * (o.x * o.z - o.w * o.y)
    # desloc = 0.05

    # arm.simpleMove([desloc*x, desloc*y, desloc*z], direction=0)

    rospy.signal_shutdown("grabbed cork debug stop")


    
    

def test():
    global positions, arm

    # arm.setSpeed(0.1)
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([p.x, p.y, p.z], [-0.471886915591, 0.0268562771098, 0.859489799629, -0.194624544454])

    # arm.jointGoal(positions['out_of_camera_pos'])
    # arm.simpleRotate([pi/2, 0, 0])
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([0.450731189437, 0.590359096664 + 0.02, p.z], [o.x, o.y, o.z, o.w])
    # p = arm.getPose().position
    # o = arm.getPose().orientation
    # arm.poseGoal([p.x, p.y, -0.0474082425519 + 0.05], [o.x, o.y, o.z, o.w])

    # arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", "init_calibration_pos")

    # o = arm.getPose().orientation
    # x = 1 - 2 * (o.y * o.y + o.z * o.z)
    # y = 2 * (o.x * o.y + o.w * o.z)
    # z = 2 * (o.x * o.z - o.w * o.y)
    # desloc = 0.10

    # arm.simpleMove([desloc*x, desloc*y, desloc*z], direction=0)

    rospy.spin()

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    global arm, positions, position_names, test_publisher
    position_names, positions = load_positions(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    rospy.Subscriber("/cork_iris/cork_piece", PoseStamped, robot2cork)

    test_publisher = rospy.Publisher('cork_iris/grabbing_position', PoseStamped, queue_size=1)

    
    
    arm = ArmControl(rospy)
    arm.setSpeed(0.1)

    parseParams(sys.argv[1:])


if __name__ == "__main__":
    main()