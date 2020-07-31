#!/usr/bin/env python
import rospy, socket, sys
from cork_iris.msg import ArmCommand
from cork_iris.srv import ControlArm

def main():

    rospy.init_node('arm_command_sender', anonymous=False)


    # cmd_pub = rospy.Publisher('cork_iris/control', ArmCommand, queue_size=1)
    # arguments = sys.argv[1:]
    # data = ArmCommand()
    # data.command = arguments
    # rospy.loginfo(data.command)
    # while not rospy.is_shutdown():
    #     connections = cmd_pub.get_num_connections()
    #     if connections > 0:
    #         cmd_pub.publish(data)
    #         rospy.loginfo('Command sent')
            # break

    arguments = sys.argv[1:]
    rospy.wait_for_service('/cork_iris/control_arm', timeout=2.5)
    try:
        control_arm = rospy.ServiceProxy('/cork_iris/control_arm', ControlArm)
        feedback = control_arm(arguments)
        rospy.loginfo("Command sent successfully") if feedback.success else rospy.logerr("Command returned with an error")
        rospy.loginfo("\n" + feedback.output_feedback)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
                
if __name__ == '__main__':
    main()