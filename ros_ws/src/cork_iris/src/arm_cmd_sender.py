#!/usr/bin/env python
import rospy, socket, sys
from cork_iris.msg import ArmCommand

def main():

    rospy.init_node('arm_command_sender', anonymous=False)
    cmd_pub = rospy.Publisher('cork_iris/control', ArmCommand, queue_size=1)
    arguments = sys.argv[1:]
    data = ArmCommand()
    data.command = arguments
    rospy.loginfo(data.command)
    while not rospy.is_shutdown():
        connections = cmd_pub.get_num_connections()
        if connections > 0:
            cmd_pub.publish(data)
            rospy.loginfo('Command sent')
            break
                
if __name__ == '__main__':
    main()