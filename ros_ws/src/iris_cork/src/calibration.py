
import rospy, os, shutil
from sensor_msgs.msg import Image
from easy_handeye.srv import TakeSample, ComputeCalibration
from std_srvs.srv import Empty

class Calibration:

    DEFAULT_HANDEYE_NAMESPACE = '/easy_handeye_eye_on_base'
    CALIBRATION_FILEPATH = '~/.ros/easy_handeye' + DEFAULT_HANDEYE_NAMESPACE + ".yaml"
    
    def __init__(self, basedir):
        rospy.loginfo("[CORK-IRIS] Initializing hand to eye calibration module!")
        self.basedir = basedir
        self.aruco_sub = rospy.Subscriber("/aruco_tracker/result", Image, self._on_aruco_result, queue_size=1)

    def _on_aruco_result(self, data):
        ''' Aruco tracker result needs to be subscriber for the calibration module to work '''
        pass


    def save_calibration_to_basedir(self):
        ''' Copies the saved calibration file in the calibration filepath to the specified basedir.
            This function should only be called if the calibration is already saved in the .ros dir'''
        try:
            src = os.path.expanduser("~") + self.CALIBRATION_FILEPATH[1:]
            dest = self.basedir+"/yaml/easy_handeye_eye_on_base.yaml"
            shutil.copyfile(src, dest)
        except Exception as e:
            rospy.logerr("[CORK-IRIS] Error while saving file to '" + dest + "'")
            rospy.logerr(e)

    def load_calibration_file(self, filename):
        ''' Loads a file from the yaml specified base directory with name <filename> and adds it to the default calibration dir'''
        try:
            src = self.basedir+"/yaml/"+filename
            dest = os.path.expanduser("~") + self.CALIBRATION_FILEPATH[1:]
            shutil.copyfile(src, dest)
        except Exception as e:
            rospy.logerr("[CORK-IRIS] Error while loading file from '" + src + "'")
            rospy.logerr(e)
            return
        rospy.loginfo("Calibration file loaded correctly!")
    
    def print_samples(self, samples):
        print("Translation" + "\t" + "Rotation")
        for sample in samples:
            print("x: " + str(sample.translation.x)[:8] + "\t" + str(sample.rotation.x)[:8])
            print("y: " + str(sample.translation.y)[:8] + "\t" + str(sample.rotation.y)[:8])
            print("z: " + str(sample.translation.z)[:8] + "\t" + str(sample.rotation.z)[:8])
            print("=========================================")

    def take_sample(self):
        rospy.wait_for_service(self.DEFAULT_HANDEYE_NAMESPACE + '/take_sample', timeout=2.5)
        take_sample_srv = rospy.ServiceProxy(self.DEFAULT_HANDEYE_NAMESPACE + '/take_sample', TakeSample)
        vals = take_sample_srv()
        print("New sample taken: ")
        transforms = vals.samples.camera_marker_samples.transforms
        self.print_samples([transforms[len(transforms)-1]])

    def compute_calibration(self):
        
        # get sample list - /easy_handeye_eye_on_base/get_sample_list
        # chamar servico compute - /easy_handeye_eye_on_base/compute_calibration
        rospy.wait_for_service(self.DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', timeout=2.5)
        compute_calibration_srv = rospy.ServiceProxy(self.DEFAULT_HANDEYE_NAMESPACE + '/compute_calibration', ComputeCalibration)
        print("Computing calibration")
        result = compute_calibration_srv()
        print("Finished calibration.")
        print(result)
        print("Saving calibration to: " + self.CALIBRATION_FILEPATH + " and " + self.basedir + "/yaml")
        rospy.wait_for_service(self.DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', timeout=2.5)
        save_calibration = rospy.ServiceProxy(self.DEFAULT_HANDEYE_NAMESPACE + '/save_calibration', Empty)
        save_calibration()
        
        self.save_calibration_to_basedir()
    

