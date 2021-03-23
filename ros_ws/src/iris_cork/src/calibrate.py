#!/usr/bin/env python
import rospy, rospkg, os, shutil, sys, csv
from math import pi
from easy_handeye.srv import TakeSample, ComputeCalibration
from std_srvs.srv import Empty
from sensor_msgs.msg import Image

from sami.arm import Arm, ArmMotionChain, EzPose

DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'
CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
BASEDIR = rospkg.RosPack().get_path('iris_cork')


def parseRotationArgs(args):
    for i in range(0, len(args)):
        args[i] = eval(args[i].replace('pi', str(pi)))
    return args


def parseActionlist(filename):
    try:
        action_file = open(filename, 'r')
    except Exception as e:
        try:
            action_file = open(BASEDIR + "/actionlists/" + filename)
        except OSError:
            print(e)
            sys.exit(0)
    
    csv_reader = csv.reader(action_file, delimiter=',')
    actions = []
    line_nr = 0
    for row in csv_reader:
        if(line_nr != 0):
            if("[" in row[2]):
                arguments = parseRotationArgs(row[2].replace("[", "").replace("]", "").split(" "))
            else: arguments = row[2]
            actions.append((row[0], row[1], arguments, row[3] == 'True'))
        line_nr += 1

    return actions


def buildArmMotionChain(actions):
    chain = ArmMotionChain()

    for action in actions:
        reps, cmd, args, sample = action

        for i in range(0, int(reps)):
            if "jointGoal" in cmd:
                chain.joints_name(args)
            elif "rotate" in cmd:
                chain.pose_relative(dpose=EzPose(roll=args[0], pitch=args[1], yaw=args[2]))
            
            if(sample):
                chain.func(take_sample)

    return chain


def save_calibration_to_basedir():
    ''' Copies the saved calibration file in the calibration filepath to the specified basedir.
        This function should only be called if the calibration is already saved in the .ros dir'''
    try:
        src = os.path.expanduser("~") + CALIBRATION_FILEPATH[1:]
        dest = BASEDIR + "/yaml/easy_handeye_eye_on_base.yaml"
        shutil.copyfile(src, dest)
    except Exception as e:
        rospy.logerr("[CORK-IRIS] Error while saving file to '" + dest + "'")
        rospy.logerr(e)


def load_calibration_file(filename):
    ''' Loads a file from the yaml specified base directory with name <filename> and adds it to the default calibration dir'''
    try:
        src = BASEDIR + "/yaml/"+filename
        dest = os.path.expanduser("~") + CALIBRATION_FILEPATH[1:]
        shutil.copyfile(src, dest)
    except Exception as e:
        rospy.logerr("[CORK-IRIS] Error while loading file from '" + src + "'")
        rospy.logerr(e)
        return
    rospy.loginfo("Calibration file loaded correctly!")


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
    return True


def compute_calibration():
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', timeout=2.5)
    compute_calibration_srv = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', ComputeCalibration)
    print("Computing calibration")
    result = compute_calibration_srv()
    print("Finished calibration.")
    print(result)
    print("Saving calibration to: " + CALIBRATION_FILEPATH + " and " + BASEDIR + "/yaml")
    rospy.wait_for_service(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', timeout=2.5)
    save_calibration = rospy.ServiceProxy(DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', Empty)
    save_calibration()
    
    save_calibration_to_basedir()


def on_aruco_result(data):
    ''' Aruco tracker result needs to be subscriber for the calibration module to work '''
    pass


def main():
    rospy.init_node('calibrate', anonymous=True)
    rospy.Subscriber("/aruco_tracker/result", Image, on_aruco_result, queue_size=1)

    actionlist = 'roller_caljob.csv'
    actions = parseActionlist(actionlist)
    print('Actions Parsed')

    m_chain = buildArmMotionChain(actions)
    print('Motion Chain Created')

    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    print('Executing Motion Chain')
    arm.move_chain(m_chain)

    compute_calibration()


if __name__ == '__main__':
    main()