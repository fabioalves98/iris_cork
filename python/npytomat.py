import cv2
import numpy as np
import sys


def npytofs(npyfilename, dstfilename):
    n = np.load(open(npyfilename, "rb"))

    fs = cv2.FileStorage(dstfilename + ".ext", cv2.FILE_STORAGE_WRITE)
    fs.write("img", n)
    fs.release()


filename = sys.argv[1]
npytofs(filename, sys.argv[2])




