import argparse
import pathlib

import PySpin
import cv2
import datetime
import os
from evaluateCNN import initCamera


def main(label):

    SAVEDIR = "/media/xtreme/data/XtremeCork/captures/BagAcquisition_29102019"
    CATEGORIES = ["Back", "Belly", "SideBellyUp", "SideBellyDown"]

    savepath = os.path.join(SAVEDIR, CATEGORIES[label])

    # Create the necessary directories
    pathlib.Path(savepath).mkdir(parents=True, exist_ok=True)

    # initialize camera
    # Get system
    system = PySpin.System.GetInstance()

    # Get camera list
    cam_list = system.GetCameras()
    # num_cameras = cam_list.GetSize()

    # Multiple cameras can be added here
    # if num_cameras > 0:
    # use first camera, might add more later
    cam = cam_list.GetByIndex(0)
    initCamera(cam)

    while True:
        now = datetime.datetime.now()
        filename = savepath + ("/Acquisition-%s.jpg" % now)
        img = cam.GetNextImage()

        if img.GetImageStatus() != PySpin.IMAGE_NO_ERROR:
            print(f"Image incomplete with image status {img.GetImageStatus()}.")
        else:
            img_conv = img.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
            # cv2.imshow('image', img_conv.GetNDArray())

            img_conv.Save(filename)
            # print("Image saved at %s" % filename)

        # cam.EndAcquisition()
        # if cv2.waitKey(1) == 27:
        #     break

    cam.EndAcquisition()
    cam.DeInit()

    del cam
    cam_list.Clear()
    system.ReleaseInstance()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture dataset with labels")
    g = parser.add_mutually_exclusive_group()
    g.add_argument('--back', '--ba', dest='label', action='store_const', const=0, help='label Back')
    g.add_argument('--belly', '--be', dest='label', action='store_const', const=1, help='label Belly')
    g.add_argument('--sideBellyUp', '--sbu', dest='label', action='store_const', const=2, help='label SideBellyUp')
    g.add_argument('--sideBellyDown', '--sbd', dest='label', action='store_const', const=3, help='label SideBellyDown')
    g.add_argument('--sideBellyLeft', '--sbl', dest='label', action='store_const', const=4, help='label SideBellyLeft')
    g.add_argument('--sideBellyRight', '--sbr', dest='label', action='store_const', const=5, help='label SideBellyRight')
    args = parser.parse_args()
    main(args.label)
