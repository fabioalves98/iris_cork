#!/usr/bin/env python
import rospy, rospkg, os

from gazebo_msgs.srv import SpawnModel
from urdf_parser_py.urdf import URDF, Box, Pose, Joint, Link, Visual, Material, Color
from visualization_msgs.msg import Marker

from scene import CorkScene

BASE_DIR = rospkg.RosPack().get_path('iris_cork')


def gazeboSpawnModel():
    # Open and edit URDF model
    model = URDF.load(BASE_DIR + '/urdf/object.urdf')

    box_link = model.links[0]
    box_link.visuals[0].geometry (Box((0.5, 0.03, 0.03)))
    box_link.collisions[0].geometry = Box((0.5, 0.03, 0.03))

    model.save(BASE_DIR + '/urdf/object.urdf')

    # Spawm model
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name='cork_piece',
        model_xml=open(BASE_DIR + '/urdf/object.urdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=Pose(),
        reference_frame='world'
    )


def dynamicURDFModel():
    # Get model from parameter server
    description = rospy.get_param('/robot_description')
    model = URDF.from_xml_string(description)

    # Cork joint creation
    cork_joint_origin = Pose((0, 0, 0), (0, 0, 0))
    cork_joint = Joint('cork_joint', 'gripper_link', 'cork_link', 'fixed', origin=cork_joint_origin)
    
    # Cork_link creation
    cork_link_geometry = Box((0.04, 0.04, 0.3))
    cork_link_material = Material("LightGrey", Color(0.7, 0.7, 0.7, 1.0))
    cork_joint_origin = Pose((0.15, 0, 0), (0, 0, 0))
    cork_link_visuals = Visual(cork_link_geometry, cork_link_material, cork_joint_origin)
    cork_link = Link('cork_link', cork_link_visuals)

    URDF.add_link(model, cork_link)
    URDF.add_joint(model, cork_joint)

    # Convert to XML and update parameter server with new URDF
    new_description = model.to_xml_string()
    rospy.set_param('/robot_description', new_description)

    # Restart MoveIt
    os.system("rosnode kill /robot_state_publisher")
    os.system("rosnode kill /move_group")



def planningSceneModel():
    scene = CorkScene()
    print('detach - ', scene.detachCorkObject())
    print('remove - ', scene.removeCorkObject())
    print('add - ', scene.addCorkObject())
    print('attatch - ', scene.attachCorkObject())


def main():
    rospy.init_node('model', anonymous=False)

    # gazeboSpawnModel()
    # dynamicURDFModel()
    planningSceneModel()

    rospy.spin()

if __name__ == "__main__":
    main()