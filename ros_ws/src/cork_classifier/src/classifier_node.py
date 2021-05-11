#!/usr/bin/env python
import rospy, socket, sys, struct, ctypes, math, cv2, numpy
import rospkg
import os

from cork_classifier.srv import ClassifyCork
from Classifier import classify
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

name = 'belly_right'
img_id = 100

MODEL = "produtech_model.h5"
rp = rospkg.RosPack()
BASE_PATH = rp.get_path('cork_classifier')
MODEL_PATH = BASE_PATH + "/models/" + MODEL
print(MODEL_PATH)



def getCorkBoundingRect(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,250,255,0)

    kernel = numpy.ones((2,2), numpy.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)
    # thresh = cv2.dilate(thresh, kernel, iterations=2)

    max_cont = None
    max_area = 0

    contours, hierarchy = cv2.findContours(thresh, 1,  cv2.CHAIN_APPROX_SIMPLE)
    conts = []
    for c in contours:
        if(cv2.contourArea(c) > 100 and cv2.contourArea(c) < 30000):
            if cv2.contourArea(c) > max_area:
                max_cont = c
                max_area = cv2.contourArea(c)
            conts.append(c)

    cnt = cv2.minAreaRect(max_cont)
    box = cv2.boxPoints(cnt)
    box = numpy.intp(box)

    return box

def unskewImage(img, rect):

    tl = rect[0]
    tr = rect[1]
    br = rect[2]
    bl = rect[3]

    widthA = math.sqrt(math.pow(br[0] - bl[0], 2) + (math.pow(br[1] - bl[1], 2)))
    widthB = math.sqrt(math.pow(tr[0] - tl[0], 2) + (math.pow(tr[1] - tl[1], 2)))
    maxWidth = max(int(widthA), int(widthB))

    heightA = math.sqrt(math.pow(tr[0] - br[0], 2) + (math.pow(tr[1] - br[1], 2)))
    heightB = math.sqrt(math.pow(tl[0] - bl[0], 2) + (math.pow(tl[1] - bl[1], 2)))
    maxHeight = max(int(heightA), int(heightB))

    pts1 = numpy.float32([tl, tr, br, bl])
    pts2 = numpy.float32([[0,0], [maxWidth-1,0], [maxWidth-1, maxHeight-1], [0,maxHeight-1]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img,M,(maxWidth-1,maxHeight-1))

    return dst


''' Apply contours etc. Get a bounding rectangle of the cork and unskew it. Finally get the desired pixels from the
    image_raw image.'''
def getClassifiableCorkImage(img):
    
    box = getCorkBoundingRect(img)
    
    data = rospy.wait_for_message("camera/rgb/image_raw", Image, timeout=1)    
    bridge = CvBridge()
    raw_img = bridge.imgmsg_to_cv2(data, 'bgr8')
    unskewed = unskewImage(raw_img, box)

    global img_id

    if unskewed.shape[0] < unskewed.shape[1]:
        unskewed = cv2.rotate(unskewed, cv2.cv2.ROTATE_90_CLOCKWISE) 

    # print(cv2.imwrite(BASE_PATH + '/img/' + name + '_' + str(img_id) + '.jpg', unskewed))
   
    # img_id += 1
    # cv2.imshow("image", unskewed)
    # cv2.waitKey()

    return unskewed


def classify_cork_piece(data):

    data = data.cork_cloud
    br = CvBridge()
    img = br.imgmsg_to_cv2(data, 'bgr8')
    img[numpy.where((img==[0,0,0]).all(axis=2))] = [255, 255, 255]

    img = getClassifiableCorkImage(img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    (classification_category, classification_accuracy) = classify(gray, MODEL_PATH)
    
    print(classification_accuracy, classification_category)

    return [classification_category, classification_accuracy]
    # return ['', 0.0]
                    
if __name__ == '__main__':
    
    try:
        rospy.init_node('cork_classification', anonymous=False)  

        s = rospy.Service("classify_cork", ClassifyCork, classify_cork_piece)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    print("ok")

    # for i in range(0, 179):

