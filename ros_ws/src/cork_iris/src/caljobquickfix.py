#!/usr/bin/env python
import rospy, rospkg
from sensor_msgs.msg import JointState


def reorder_caljob(filename, default_joint_order, published_joint_order):
    f = open(filename, "r")
    lines = f.readlines()

    new_lines = list(lines)
    for i in range(0, len(lines)):
        if 'joint_values:' in lines[i]:
            for j in range(0, 6):
                joint_value = lines[i+(j+1)]
                original_joint = published_joint_order[j]
                new_idx = default_joint_order.index(original_joint)
                new_lines[i+(new_idx+1)] = joint_value

    f.close()

    file_write = open(filename, "w")

    for l in new_lines:
        file_write.write(l)
    file_write.close()


def main():

    rospy.init_node('caljob_joint_reorder', anonymous=True)
    joint_state = rospy.wait_for_message("/joint_states", JointState)    
    
    '''
    SIM_JOINT_ORDER = ['shoulder_pan_joint', 
                       'shoulder_lift_joint', 
                       'elbow_joint', 
                       'wrist_1_joint', 
                       'wrist_2_joint', 
                       'wrist_3_joint', 
                       'left_finger_joint',
                       'right_finger_joint']
    '''

    DEFAULT_JOINT_ORDER = ['shoulder_pan_joint', 
                       'shoulder_lift_joint', 
                       'elbow_joint', 
                       'wrist_1_joint', 
                       'wrist_2_joint', 
                       'wrist_3_joint']

    rospack = rospkg.RosPack()
    path = rospack.get_path('cork_iris')    
    CALJOB_FILENAME = path + '/yaml/ur10_kinect_caljob.yaml'


    ## This array contains the published order of the joints. Should be the original order
    ## from which the caljob is created
    published_joint_names = joint_state.name
    reorder_caljob(CALJOB_FILENAME, DEFAULT_JOINT_ORDER, published_joint_names)
    # rospy.spin()

if __name__ == '__main__':
    main()

