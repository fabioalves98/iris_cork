#!/usr/bin/env python

import sys, copy, time, yaml
import rospy, rospkg, rosparam
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
from ArmControl import ArmControl

arm = None

CORK_IRIS_BASE_DIR = rospkg.RosPack().get_path('cork_iris')
DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'
CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"

positions = {}

def load_positions(path):
    '''Returns tuple with keys of all possible positions and the dict with the positions previously
       saved in a yaml file'''

    print(CORK_IRIS_BASE_DIR)
    data = rosparam.load_file(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    k = data[0][0].keys()

    return k, data[0][0]

def parseRotationArgs(args):

    for i in range(0, len(args)):
        args[i] = eval(args[i].replace('pi', str(pi)))
    return args

def parseParams(args):
    global positions
    print("PARSING ARGS")
    try:
        print (args)
        # X > 0 - Forward
        # y > 0 - Left
        # Z > 0 - Upwards
        if("move" in args[0]):
            arm.simpleMove([float(args[1]), float(args[2]), float(args[3])], pi/4)
        
        elif("rotate" in args[0]):
            args = parseRotationArgs(args[1:4])
            arm.simpleRotate([args[0], args[1], args[2]])
        
        elif("initial" in args[0]):
            arm.jointGoal(positions['init_live_pos'])
        
        elif("caljob" in args[0]):
            caljob()
        elif("grip" in args[0]):
            arm.grip()
        elif("release" in args[0]):
            arm.release()
        else:
            print("Usage: rosrun cork_iris arm_control.py <command> <command_params>")
            print("Available commands:")
            print("\tmove   <x> <y> <z> -> Simple cartesian movement relative to last position")
            print("\trotate <x> <y> <z> -> Simple rotation relative to last position")
            print("\tinitial            -> Joint goal to the default initial position")
            print("\tcaljob             -> Calibration job")
            print("\tgrip               -> Grip the gripper")
            print("\trelease            -> Release the gripper")

    except Exception as e:
        if len(args) == 0:
            test()
        else:
            print(e)
            print("Wrong arguments provided! Check the help command")


def caljob():
    global positions
    init_pos = positions['init_live_pos']
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
        take_sample()
    
    arm.jointGoal(init_pos)
    arm.simpleMove([-0.03, 0, -0.03], pi/4)

    for i in range(3):
        arm.simpleRotate([0, 0, -pi/9])
        take_sample()
    
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
    global positions, arm

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
    # arm.setSpeed(0.1)
    # arm.jointGoal(positions['out_of_camera_pos'])
    # arm.simpleMove([0, 0, -0.05], pi/4)
    # arm.saveJointPosition(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml", "test_position")
    
    # arm.jointGoal(positions['out_of_camera_pos'])

    arm.jointGoal(positions['vert_pick_pos'])
    
    print(arm.getPose())

    # arm.grip()
    # print(arm.jointValues())

    # rospy.spin()

    # arm.jointGoal(positions['out_of_camera_pos'])


def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    global arm, positions
    position_names, positions = load_positions(CORK_IRIS_BASE_DIR + "/yaml/positions.yaml")
    rospy.Subscriber("/aruco_tracker/result", Image, aruco_callback)
    rospy.Subscriber("/cork_iris/cork_center", Point, robot2cork)
    
    arm = ArmControl(rospy)

    parseParams(sys.argv[1:])


if __name__ == "__main__":
   main() 