import tf2_ros, tf2_geometry_msgs
import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import PoseStamped, Quaternion, Point


def posePositionToArray(position):
    return [position.x, position.y, position.z]

def poseOrientationToArray(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]

def arrayToPosePosition(array):
    p = Point(*array)
    return p

def arrayToPoseOrientation(array):
    o = Quaternion(*array)
    return o

def getTransform(src_tf, dest_tf):
    ''' Uses tf2 buffer lookup transform to return the transform from src_tf to dest_tf'''
    print("GET TRANSFORM BEING CALLED")
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

def keep_going(text):
    goon = raw_input("[ACTION] -> " + text + "['n' to stop]")
    if('n' in goon):
        return False
    return True

def setPCLCorkParameter(params={"live" : "true", "type" : "4"}):
    ''' Updates the pcl_cork parameters using "params" values'''
    client = dynamic_reconfigure.client.Client("pcl_cork", timeout=30)
    client.update_configuration(params)

def newPoseStamped(position=[0,0,0], orientation=[0,0,0,1], frame_id="base_link"):

    pose = PoseStamped()
    pose.pose.position = Point(*position)
    pose.pose.orientation = Quaternion(*orientation)
    pose.header.frame_id = frame_id

    return pose



if __name__=="__main__":
    rospy.init_node('helper_functions', anonymous=False)
    rospy.spin()