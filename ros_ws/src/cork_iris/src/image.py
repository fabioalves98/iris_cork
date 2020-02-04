#!/usr/bin/env python
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()

pub = rospy.Publisher('/encoded', Image, queue_size=10)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="mono16")
    cv2.imshow('image',cv_image)

    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="mono16")
    pub.publish(image_message)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('encode', anonymous=True)

    
    rospy.Subscriber("/kinect/ir/image_raw", Image, callback)
    rate = rospy.Rate(10) # 10hz
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()