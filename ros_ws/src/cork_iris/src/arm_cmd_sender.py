#!/usr/bin/env python
import rospy, socket, sys
from cork_iris.msg import ArmCommand
from cork_iris.srv import ControlArm


def callControlArmService(arguments, printFeedback=True):
    rospy.wait_for_service('/cork_iris/control_arm', timeout=2.5)
    try:
        control_arm = rospy.ServiceProxy('/cork_iris/control_arm', ControlArm)
        feedback = control_arm(arguments)
        rospy.loginfo("Command sent successfully") if feedback.success else rospy.logerr("Command returned with an error")
        if printFeedback: rospy.loginfo(feedback.output_feedback)
        return feedback.output_feedback
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():

    rospy.init_node('arm_command_sender', anonymous=False)

    arguments = sys.argv[1:]
    if arguments:
        # rospy.wait_for_service('/cork_iris/control_arm', timeout=2.5)
        # try:
        #     control_arm = rospy.ServiceProxy('/cork_iris/control_arm', ControlArm)
        #     feedback = control_arm(arguments)
        #     rospy.loginfo("Command sent successfully") if feedback.success else rospy.logerr("Command returned with an error")
        #     rospy.loginfo(feedback.output_feedback)
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        callControlArmService(arguments)
    
    helpStr = callControlArmService(["help"], printFeedback=False)
    helpStr = helpStr[helpStr.find("\n"):]
    while not rospy.is_shutdown():
        rospy.loginfo(helpStr)
        arguments = raw_input("Insert command: ").split(" ")
        if("exit" in arguments):
            break
        callControlArmService(arguments)
        rospy.sleep(1.0)
    
    # rospy.spin()
                
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass