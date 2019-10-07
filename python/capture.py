#!/usr/bin/python
import freenect
import numpy as np
import cv2

def get_video():
    array,_ = freenect.sync_get_video()
    return array

def get_depth():
    array,_ = freenect.sync_get_depth()
    return array

def pretty_depth(depth):
    np.clip(depth, 0, 2**10-1, depth)
    depth >>=2
    depth=depth.astype(np.uint8)
    return depth
if __name__ == "__main__":
    num = 0
    while 1:
        frame = get_depth()
        frame = pretty_depth(frame)

        frameRGB = get_video()
        #frameRGB = pretty_depth(frameRGB)

        cv2.imshow('IR image',frame)
        cv2.imshow('RGB image',frameRGB)

        k = cv2.waitKey(5) & 0xFF
        if k == ord('q'):

            np.save('depth-' + str(num), frame)
            np.save('rgb-' + str(num), frameRGB)

            print ("Photo")

            num += 1
            
        if k == 27:
            break
        
    cv2.destroyAllWindows()