#import freenect
import numpy as np
import cv2
from scipy.interpolate import splprep, splev



def smoothen_contours(contours):

    smoothened = []
    for contour in contours:
        x,y = contour.T
        # num py to normal array
        x = x.tolist()[0]
        y = y.tolist()[0]
        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splprep.html
        tck, u = splprep([x,y], u=None, s=1.0, per=1)
        # https://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linspace.html
        u_new = np.linspace(u.min(), u.max(), 25)
        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splev.html
        x_new, y_new = splev(u_new, tck, der=0)
        # normal array back to numpy
        res_array = [[[int(i[0]), int(i[1])]] for i in zip(x_new,y_new)]
        smoothened.append(np.asarray(res_array, dtype=np.int32))

    return smoothened

if __name__ == "__main__":

    # cv2.imshow('IR image',np.load(open("depth.out", "r")))
    # cv2.imshow('RGB image',np.load(open("rgb.out.npy", "r")))
    num = 1
    font = cv2.FONT_HERSHEY_COMPLEX

    # while 1:

    try:
        ir = np.load(open("firstShots/depth-" + str(num) + ".npy", "rb"))
        rgb = np.load(open("firstShots/rgb-" + str(num) + ".npy", "rb"))
        # cv2.imshow('IR image', ir)
        # cv2.imshow('RGB image',)
    except Exception as e:
        print(e)
        pass


    # for i in range(0, len(ir)):
    #     for j in range(0, len(ir[0])):
    #         if(ir[i][j] < 154):
    #             ir[i][j] = 0
    #         elif(ir[i][j] > 185):
    #             ir[i][j] = 255
    #         else:
    #             ir[i][j] = (ir[i][j] - 154) * (255 / 29)

    # edges = cv2.Canny(rgb,200,300)
    # cv2.imshow('e', edges)
    # cv2.waitKey(0)

    ## TESTAR COM VARIOS VALORES!!
    # threshold_val = 180
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    ret,th = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)


    # hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    # cv2.imshow('image', hsv)
    
    ## "morphological close -> tira rasguras de certos contornos da imagem"
    ## Testar possiveis argumentos - kernel p.ex - nao pareceu util
    # kernel = np.ones((15,15),np.uint8)
    # dilate = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, 3)
    
    ## Sendo que e possivel isolar cada contorno, conseguimos detetar cada traco individualmente
    _,contours,hierarchy = cv2.findContours(th, 2, 1)

    filtered_contours = []
    min_area = 1000
    max_area = 8000 

    for c in contours:
        area = cv2.contourArea(c)
        if area > min_area and area < max_area:
            filtered_contours.append(c)
    
    # filtered_contours = smoothen_contours(filtered_contours)
    
    for c in filtered_contours:
        # approx = cv2.approxPolyDP(c, 0.01*cv2.arcLength(c, True), True)
        # hull = cv.convexHull(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        cv2.drawContours(rgb, [box], 0, (0, 255, 0), 1)
        x = box.ravel()[0]
        y = box.ravel()[1]
        if(len(box) == 4):
            cv2.putText(rgb, "traco", (x,y), font, 1, (255, 255, 255))
        


    # cv2.drawContours(rgb, filtered_contours, -1, (0,255,0), 1)


    cv2.imshow('IR image', rgb)
    # cv2.imshow('image', rgb)

    cv2.waitKey(0)

            
        # k = cv2.waitKey(5) & 0xFF

        # if k == 27:
        #     break
        # elif k == ord('n'):
        #     num += 1
        
    cv2.destroyAllWindows()






