#!/usr/bin/env python
import sys, time, csv
import rospy, rospkg, rosparam, tf
import tf2_geometry_msgs
import geometry_msgs.msg
from math import pi, cos, sin

from geometry_msgs.msg import Point, TransformStamped, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from helpers import newPoseStamped, samiPoseService, samiMoveService, samiGripperService, \
    samiAliasService, keep_going, setPCLCorkParameter, getTransform

from iris_cork.msg import CorkInfo


def computeCorkGrabPositions(trans):
    '''Gets a position -0.15 meters behind the original cork piece. this should be the 
       grabbing position'''

    aux = newPoseStamped([-0.15, 0, 0], frame_id="base_link")

    grab_pose_1 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    inverted_angle = 0

    if grab_pose_1.pose.position.z < trans.transform.translation.z:
        rospy.loginfo("Inverting cork piece orientation!")
        inverted_angle = pi

    inv_quaternion = tf.transformations.quaternion_multiply(
        (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w),
        quaternion_from_euler(pi, inverted_angle, 0))

    trans.transform.rotation = Quaternion(*inv_quaternion)
    aux.pose.position.x = -0.15

    grab_pose_1 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    aux.pose.position.x = -0.070
    grab_pose_2 = tf2_geometry_msgs.do_transform_pose(aux, trans)

    # TODO: fix this bug
    grab_pose_1.pose.position.x = grab_pose_1.pose.position.x + 0.020
    grab_pose_2.pose.position.x = grab_pose_2.pose.position.x + 0.020

    grab_pose_1.pose.position.y = grab_pose_1.pose.position.y - 0.015
    grab_pose_2.pose.position.y = grab_pose_2.pose.position.y - 0.015
    
    return (grab_pose_1, grab_pose_2)


def grab_cork(cork_pose, cork_grab_pose):
    '''Grab a cork piece given its pose and a 15cm distant pose'''
    if(not keep_going("grab_cork")):
        return

    samiPoseService(cork_grab_pose)
   
    if(not keep_going('go towards cork')):
        return 
    
    samiMoveService([0.08, 0, 0] + [0, 0, 0])

    if(not keep_going('grip')):
        return 

    samiGripperService('grip')

    time.sleep(0.5)

    samiMoveService([-0.15, 0, 0] + [0, 0, 0])
    
    samiAliasService('out_of_camera')

    if(not keep_going("deliver")):
        return 

    samiAliasService('delivery')

    time.sleep(0.5)

    samiGripperService('release')

    samiAliasService('out_of_camera')
     

def main():
    rospy.init_node('grab_cork', anonymous=False)

    grab_pose_pub = rospy.Publisher('cork_iris/grabbing_position', PoseStamped, queue_size=1)

    rospy.loginfo("Grabbing routine started")

    setPCLCorkParameter()

    cork_info_sub = rospy.Subscriber('cork_iris/cork_info', CorkInfo, queue_size=1)

    while not rospy.is_shutdown():
        # trans = None
        # tries = 2
        
        # while not trans and tries > 0:
        #     trans = getTransform("base_link", "cork_piece")
        #     tries -= 1
        
        # setPCLCorkParameter({"live" : "false"})

        # if trans == None:
        #     return "No Cork Piece found"

        grab1, grab2 = computeCorkGrabPositions(trans)
        grab_pose_pub.publish(grab1)
        
        grab_cork(grab2.pose, grab1.pose)
        
        rospy.loginfo("Ended grabbing routine")
        setPCLCorkParameter()

        break


if __name__ == "__main__":
    main()