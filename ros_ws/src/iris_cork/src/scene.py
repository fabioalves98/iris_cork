#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class CorkScene:
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.robot = moveit_commander.RobotCommander()
        self.object_name = 'cork_piece'
        self.eef_link = self.move_group.get_end_effector_link()
        print(self.eef_link)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=10):
        box_name = self.object_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def addCorkObject(self, timeout=10):
        box_name = self.object_name
        scene = self.scene

        box_pose = PoseStamped()
        box_pose.header.frame_id = "gripper_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.10
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.5))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def removeCorkObject(self, timeout=10):
        box_name = self.object_name
        scene = self.scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    def attachCorkObject(self, timeout=10):
        box_name = self.object_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link

        # grasping_group = 'manipulator'
        # touch_links = robot.get_link_names(group=grasping_group)
        touch_links = ['flange', 'gripper_link', 'left_finger_link', 'right_finger_link']
        print(touch_links)

        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
    

    def detachCorkObject(self, timeout=4):
        box_name = self.object_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)