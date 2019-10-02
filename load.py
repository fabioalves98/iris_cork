#!/usr/bin/python
import freenect
import numpy as np
import cv2

if __name__ == "__main__":

    # cv2.imshow('IR image',np.load(open("depth.out", "r")))
    # cv2.imshow('RGB image',np.load(open("rgb.out.npy", "r")))
    num = 0


    while 1:

        try:
            ir = np.load(open("depth-" + str(num) + ".npy", "r"))
            rgb = np.load(open("rgb-" + str(num) + ".npy", "r"))
            # cv2.imshow('IR image', ir)
            # cv2.imshow('RGB image',)
        except Exception:
            break
        
        ## TESTAR COM VARIOS VALORES!!
        threshold_val = 180
        ret,th = cv2.threshold(ir,175,255, 1)

        ## "morphological close -> tira rasguras de certos contornos da imagem"
        ## Testar possiveis argumentos - kernel p.ex - nao pareceu util
        # kernel = np.ones((15,15),np.uint8)
        # dilate = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, 3)
        
        ## Sendo que e possivel isolar cada contorno, conseguimos detetar cada traco individualmente
        contours,hierarchy = cv2.findContours(th, 2, 1)
        cv2.drawContours(ir, contours, -1, (0,255,0), 3)


        cv2.imshow('IR image', ir)


        
        k = cv2.waitKey(5) & 0xFF

        if k == 27:
            break
        elif k == ord('n'):
            num += 1
        
    cv2.destroyAllWindows()