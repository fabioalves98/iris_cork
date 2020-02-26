import numpy as np
import cv2, PIL, os
from cv2 import aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)
imboard = board.draw((3000, 2000))
cv2.imwrite("chessboard.png", imboard)
