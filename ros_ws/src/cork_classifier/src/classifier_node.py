#!/usr/bin/env python
import rospy, socket, sys, struct, ctypes
import rospkg 

from cork_classifier.srv import ClassifyCork
# from Classifier import classify
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# # RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "res")
# # print(RESOURCES)

# # def main():
# #     # Choose a model from "models" folder in resources
# #     model = os.path.join(RESOURCES, "models", "conv-32-drop25-dense-128-drop50-64-drop50-1572285048.h5")
# #     print(model)

# #     # Demo images
# #     images_path = []
# #     try:
# #         images_path = [os.path.join(os.path.join(RESOURCES, "demo_images"), f) for f in os.listdir(os.path.join(RESOURCES, "demo_images"))]
# #     except Exception:
# #         pass

# #     for img in images_path:
# #         # Read image from file
# #         image = cv2.imread(img)
# #         cv2.imshow("Demo Classification v0.1", image)

# #         # image = image to classify, model = path to model,
# #         # gpu = enable GPU for a single classification (for better performance enable on main function)
# #         (classification_category, classification_accuracy) = classify(image, model)




MODEL = "conv-32-drop25-dense-128-drop50-64-drop50-1572285048.h5"
rp = rospkg.RosPack()
BASE_PATH = rp.get_path('cork_classifier')
MODEL_PATH = BASE_PATH + "/models/" + MODEL


def valueToRgb(val):
    s = struct.pack('>f' ,val)
    i = struct.unpack('>l',s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000)>> 16
    g = (pack & 0x0000FF00)>> 8
    b = (pack & 0x000000FF)
    return (r, g, b)


def classify_cork_piece(data):
    print(data)

    cloud_data = rospy.wait_for_message('camera/depth_registered/points', PointCloud2, timeout=5)
        
    parsed_cloud = pc2.read_points(cloud_data, skip_nans=True)
    int_data = list(parsed_cloud)
    print (int_data[0]) #Gives first point etc
    print(valueToRgb(int_data[0][3]))
    # print(parsed_cloud["x"])
    return 1
                    
if __name__ == '__main__':
    
    try:
        rospy.init_node('cork_classification', anonymous=False)  
        print(MODEL_PATH)

        s = rospy.Service("classify_cork", ClassifyCork, classify_cork_piece)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    print("ok")