#!/usr/bin/env python

import rospy
import moveit_commander

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from HelperFunctions import getTransform, newPoseStamped
from tf.transformations import quaternion_from_euler
from math import pi


CAMERA_HEIGHT = 0.05
CAMERA_WIDTH = 0.05
CAMERA_LENGTH = 0.2

scene = None
def wait_for_state_update(object_name="", object_is_known=False, object_is_attached=False, timeout=4):
    
    start = rospy.get_time()
    seconds = rospy.get_time()
    while(seconds - start < timeout) and not rospy.is_shutdown():
        # Check if object is in attached objects
        attached_objects = scene.get_attached_objects([object_name])
        is_attached = len(attached_objects.keys()) > 0
        # Check if object is in world objects
        is_known = object_name in scene.get_known_object_names()
        if (object_is_attached == is_attached) and (object_is_known == is_known):
            return True
        
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False


## TODO: Create auxiliary function file? These functions are useful elsewhere as well
def objectToArray(obj):
    ''' Takes position or orientation objects and converts them into arrays '''
    try:
        arr = [obj.x, obj.y, obj.z, obj.w]
    except:
        arr = [obj.x, obj.y, obj.z]
    return arr



def delete_objects():
    scene.remove_world_object()
    wait_for_state_update()

def main():
    global scene

    rospy.init_node('setup_scene', anonymous=False)
    rospy.on_shutdown(delete_objects)
    
    scene = moveit_commander.PlanningSceneInterface()

    camera_transform = getTransform('base_link', 'camera_link')
    print(camera_transform)
    rospy.sleep(2)
    
    pos = objectToArray(camera_transform.transform.translation)
    ori = objectToArray(camera_transform.transform.rotation)
    camera_box = newPoseStamped(pos, ori, "base_link")
    scene.add_box("camera_box", camera_box, size=(CAMERA_WIDTH, CAMERA_LENGTH, CAMERA_HEIGHT))
    status = wait_for_state_update("camera_box", object_is_known=True)
    rospy.loginfo("Created camera box") if status else rospy.logwarn("Failed creating camera box")

    # pos[0] += 0.1
    # pos[1] += 0.1
    # camera_plane = newPoseStamped(pos, quaternion_from_euler(0, 0, -pi/4), "base_link")
    # scene.add_box("camera_plane", camera_plane, size=(1, 0.05, 2))    
    # status = wait_for_state_update("camera_plane", object_is_known=True)
    # rospy.loginfo("Created camera plane") if status else rospy.logwarn("Failed creating camera plane")


    base_plane = newPoseStamped([0, 0, 0], quaternion_from_euler(pi/2, 0, pi/4), "base_link")
    scene.add_box("base_plane", base_plane, size=(1, 0.05, 1))
    status = wait_for_state_update("base_plane", object_is_known=True) 
    rospy.loginfo("Created base plane") if status else rospy.logwarn("Failed creating base plane")
    


    rospy.spin()


if __name__ == "__main__":
    main()