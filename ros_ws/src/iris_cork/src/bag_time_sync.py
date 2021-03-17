#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import message_filters


def tf_cb(msg):
    for i, tf in enumerate(msg.transforms):
        tf.header.stamp = rospy.Time.now()
    pub_tf.publish(msg)


def rgb_cb(msg):
    # print("RGB: ", msg.header.stamp)
    msg.header.stamp = rospy.Time.now()
    pub_rgb.publish(msg)

def depth_raw_cb(msg):
    msg.header.stamp = rospy.Time.now()
    pub_depth_raw.publish(msg)
    # print("RAW: ", msg.header.stamp)
# pub_2.publish(msg)

def depth_points_cb(msg):
    msg.header.stamp = rospy.Time.now()
    # print("POINTS: ", msg.header.stamp)
    pub_depth_points.publish(msg)

def depth_info_cb(msg):
    msg.header.stamp = rospy.Time.now()
    pub_depth_info.publish(msg)
    # print("INFO: ", msg.header.stamp)
    # pub_2.publish(msg)



rospy.init_node('hoge')

rospy.sleep(1)


pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
pub_rgb = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
pub_depth_raw = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
pub_depth_points = rospy.Publisher('/camera/depth_registered/points', PointCloud2, queue_size=1)
pub_depth_info = rospy.Publisher('/camera/depth_registered/camera_info', CameraInfo, queue_size=1)


sub_tf = rospy.Subscriber('/tf_nouse', TFMessage, tf_cb)
sub_rgb = rospy.Subscriber('/rgb/image_raw_nouse', Image, rgb_cb)
sub_depth_raw = rospy.Subscriber('/depth_registered/image_raw_nouse', Image, depth_raw_cb)
sub_depth_points = rospy.Subscriber('/depth_registered/points_nouse', PointCloud2, depth_points_cb)
sub_depth_info = rospy.Subscriber('/depth_registered/camera_info_nouse', CameraInfo, depth_info_cb)




rospy.spin()