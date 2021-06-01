#!/usr/bin/env python
import rospy
from math import radians
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Vector3, Quaternion
from tf2_geometry_msgs import do_transform_pose

import moveit_commander

from helpers import pointToList


def main():
    rospy.init_node('pm_calibration', anonymous=False)

    br = tf2_ros.TransformBroadcaster()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

    robot = moveit_commander.RobotCommander()

    ee_pose = move_group.get_current_pose()

    aux_cork_tf = TransformStamped()
    aux_cork_tf.header.frame_id = 'base_link'
    aux_cork_tf.transform.translation = ee_pose.pose.position
    aux_cork_tf.transform.rotation = ee_pose.pose.orientation

    aux_cork_pose = PoseStamped()
    aux_cork_pose.header.frame_id = 'base_link'
    aux_cork_pose.pose.position = Point(*[0.16, 0, 0])
    aux_cork_pose.pose.orientation = Quaternion(*[0, 0, 0, 1])

    # Cork Pose
    cork_pose = do_transform_pose(aux_cork_pose, aux_cork_tf)

    cork_pose_tf = TransformStamped()
    cork_pose_tf.header.stamp = rospy.Time.now()
    cork_pose_tf.header.frame_id = 'base_link'
    cork_pose_tf.child_frame_id = 'cork_pose'
    cork_pose_tf.transform.translation = Vector3(*pointToList(cork_pose.pose.position))
    cork_pose_tf.transform.rotation = cork_pose.pose.orientation

    # Delivery 90 
    aux_del_90_pose = PoseStamped()
    aux_del_90_pose.header.frame_id = 'base_link'
    aux_del_90_pose.pose.position = Point(*[0, -0.16, 0])
    aux_del_90_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(90)))

    del_90_pose = do_transform_pose(aux_del_90_pose, cork_pose_tf)

    del_90_tf = TransformStamped()
    del_90_tf.header.stamp = rospy.Time.now()
    del_90_tf.header.frame_id = 'base_link'
    del_90_tf.child_frame_id = 'del_90_pose'
    del_90_tf.transform.translation = Vector3(*pointToList(del_90_pose.pose.position))
    del_90_tf.transform.rotation = del_90_pose.pose.orientation

    # Delivery 180 
    aux_del_180_pose = PoseStamped()
    aux_del_180_pose.header.frame_id = 'base_link'
    aux_del_180_pose.pose.position = Point(*[0.16, 0, 0])
    aux_del_180_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(180)))

    del_180_pose = do_transform_pose(aux_del_180_pose, cork_pose_tf)

    del_180_tf = TransformStamped()
    del_180_tf.header.stamp = rospy.Time.now()
    del_180_tf.header.frame_id = 'base_link'
    del_180_tf.child_frame_id = 'del_180_pose'
    del_180_tf.transform.translation = Vector3(*pointToList(del_180_pose.pose.position))
    del_180_tf.transform.rotation = del_180_pose.pose.orientation


    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        cork_pose_tf.header.stamp = rospy.Time.now()
        del_90_tf.header.stamp = rospy.Time.now()
        del_180_tf.header.stamp = rospy.Time.now()

        br.sendTransform(cork_pose_tf)
        br.sendTransform(del_90_tf)
        br.sendTransform(del_180_tf)


        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()