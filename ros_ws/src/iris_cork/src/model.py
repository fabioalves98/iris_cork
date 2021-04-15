#!/usr/bin/env python
import rospy, rospkg, os

from gazebo_msgs.srv import SpawnModel
from urdf_parser_py.urdf import URDF, Box, Pose, Joint, Link, Visual, Material, Color
from visualization_msgs.msg import Marker

BASE_DIR = rospkg.RosPack().get_path('iris_cork')


def main():
    rospy.init_node('model', anonymous=False)

    # model = URDF.load(BASE_DIR + '/urdf/object.urdf')

    # box_link = model.links[0]
    # box_link.visuals[0].geometry = Geometry(Box((0.5, 0.03, 0.03)))
    # box_link.collisions[0].geometry = Geometry(Box((0.5, 0.03, 0.03)))

    # model.save(BASE_DIR + '/urdf/object.urdf')

    # # Spawm model
    # spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    # spawn_model_client(
    # model_name='cork_piece',
    # model_xml=open(BASE_DIR + '/urdf/object.urdf', 'r').read(),
    # robot_namespace='/foo',
    # initial_pose=Pose(),
    # reference_frame='world'
    # )

    description = rospy.get_param('/robot_description')

    # for link in model.links:
    #     print(link.name)

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


    print(model)

    new_description = model.to_xml_string()
    print(rospy.set_param('/robot_description', new_description))

    os.system("rosnode kill /robot_state_publisher")
    os.system("rosnode kill /move_group")

    # m_x_pub = rospy.Publisher('cork_marker', Marker, queue_size=10)

    # m_x = Marker()
    # m_x.header.frame_id = "base_link"
    # m_x.type = m_x.ARROW
    # m_x.action = m_x.ADD
    # m_x.scale = Vector3(*[0.2, 0.02, 0.02])

    # rot = quaternion_from_euler(0, 0, radians(-90))
    # m_x_ori = quaternion_multiply(orientation_to_list(ee_ori), rot)
    # m_x.pose.orientation = list_to_orientation(m_x_ori)

    # m_x.pose.position  = Point(*origin)
    # m_x.color = ColorRGBA(*[1, 0, 0, 1])


if __name__ == "__main__":
    main()