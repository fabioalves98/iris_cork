#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("test_freedrive", anonymous=True)
    pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size = 1)
    print("Waiting so connection is established")
    rospy.sleep(5)

    while not rospy.is_shutdown():
        print("sending script ")
        urs = String("def prog():\nfreedrive_mode()\nend\n")
        pub.publish(urs)
        rospy.sleep(1)

if __name__ == '__main__': main()
