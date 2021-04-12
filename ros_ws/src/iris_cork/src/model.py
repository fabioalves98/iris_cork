#!/usr/bin/env python
import rospy, rospkg

from gazebo_msgs.srv import SpawnModel
from urdf_parser_py.urdf import URDF, Box, Pose
# from urdfpy import URDF, Geometry, Box
# from geometry_msgs.msg import Pose

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
    # model = URDF.from_parameter_server()
    # print(robot_description)
    # model = URDF.load(rospy.get_param('/robot_description'))

    # for link in model.links:
    #     print(link.name)

    model = URDF.from_xml_string(description)

    # print(model)

    for link in model.links:
        if link.name == 'left_finger_link':            
            link.visuals[0].geometry = Box((0.5, 0.02, 0.02))
            link.visuals[0].origin = Pose((0.3, 0, 0), (0, 0, 0))
            print(link.collisions[0])
            link.collisions[0].geometry = Box((0.5, 0.02, 0.02))
            link.collisions[0].origin = Pose((0.3, 0, 0), (0, 0, 0))
            print(link.collisions[0])


    # print(model)

    new_description = model.to_xml_string()

    # print(new_description)

    print(rospy.set_param('/robot_description', new_description))

    # print(model)
    # print(type(model))


if __name__ == "__main__":
    main()