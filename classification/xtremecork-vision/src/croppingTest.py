import time

import cv2
import numpy as np
from evaluateCNN import trainBackground, create_test_data

def detectCorkStrip(raw_image, background_model):
    """Detects and extracts the cork strip from the image, result is vertically alligned and the original angle

    :param raw_image: Image for strip detection
    :param background_model: trained background model (will be updated)
    :return: (cropped strip, angle)
    """

    # cropped the reflection from the lights
    offset = raw_image.shape[1] - 654
    img_cropped = raw_image[:, int((offset / 2)):int((raw_image.shape[1] - offset / 2))]
    img_cropped_rect = img_cropped.copy()
    # Noise reduction
    img_array = cv2.GaussianBlur(img_cropped, (7, 7), 0)
    # apply the foreground mask
    foreground = background_model.apply(img_array)

    # threshold to remove the detected shadows
    dump, foreground = cv2.threshold(foreground, 128, 255, cv2.THRESH_BINARY)
    kernel = np.ones((7, 7), np.uint8)
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow("foreground", foreground)
    # retrieve the contours
    contours, hierarchy = cv2.findContours(foreground, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # get area of each contour
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

    cv2.imshow("foreground", foreground)

    # init black images*
    corkMask = np.zeros(foreground.shape, np.uint8)
    crop = np.zeros(foreground.shape, np.uint8)
    if len(contours) > 0:
        # retrieve the biggest contour (Try to find the cork strip)
        area, biggest_contour = max(contour_sizes, key=lambda x: x[0])

        # CORK STRIP AREA THRESH = 40 000
        if area > 2000:
            rect = cv2.minAreaRect(biggest_contour)

            center, (rectW, rectH), angle = rect

            # print("center: ", center)
            # if (center[0] > (img_cropped.shape[1] / 2 + img_cropped.shape[1] * 0.15) or
            #         center[0] < (img_cropped.shape[1] / 2 - img_cropped.shape[1] * 0.15)):
            #   continue
            # if (img_cropped.shape[1] / 2 + img_cropped.shape[1] * 0.15) > center[0] > \
            #         (img_cropped.shape[1] / 2 - img_cropped.shape[1] * 0.15):
            if rectW > 400 or rectH > 400:

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                src_pts = box.astype("float32")
                # src points are assigned clockwise and depends if rectW is horizontal or vertical
                src_pts_ori = np.int0([[x[0] + (offset/2), x[1]] for x in src_pts]).astype("float32")

                if rectW < rectH:
                    # this means the width is the height and first point is bottom right
                    # check if strip reached the end
                    # if points 1 and 4 are in the image means the cork strip is over (Check X value)
                    if src_pts_ori[0][0] < (raw_image.shape[1] - offset/2 - 10) \
                            and src_pts_ori[3][0] < (raw_image.shape[1] - offset/2 - 10):
                        # cork is over
                        # print(f"point 1: {src_pts_ori[0][0]}  || point 4: {src_pts_ori[3][0]}")
                        return None, 0
                    # Normalize the angles (0º is vertical)
                    # fix angles
                    if angle < 0:
                        angle += 180
                    dst_pts = np.array([[rectW - 1, 0],
                                        [rectW - 1, rectH - 1],
                                        [0, rectH - 1],
                                        [0, 0]], dtype="float32")
                    M = cv2.getPerspectiveTransform(src_pts_ori, dst_pts)
                else:
                    # this means the width is correct and first point is bottom left
                    # check if strip reached the end
                    # if points 3 and 4 are in the image means the cork strip is over (Check X value)
                    if src_pts_ori[2][0] < (raw_image.shape[1] - offset/2 - 10) \
                            and src_pts_ori[3][0] < (raw_image.shape[1] - offset/2 - 10):
                        # print(f"point 3: {src_pts_ori[2][0]}  || point 4: {src_pts_ori[3][0]}")
                        # cork is over
                        return None, 0

                    # Normalize the angles (0º is vertical)
                    # fixing angles
                    angle += 90

                    dst_pts = np.array([[rectH - 1, rectW - 1],
                                        [0, rectW - 1],
                                        [0, 0],
                                        [rectH - 1, 0]], dtype="float32")
                    M = cv2.getPerspectiveTransform(src_pts_ori, dst_pts)

                    # swap height and width for further calculations
                    rectH, rectW = rectW, rectH

                # print bounding box corner numbers
                # count = 1
                # for point in src_pts_ori:
                #     cv2.putText(raw_image, f"{count}", (point[0], point[1]),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                #     count += 1

                # apply the rotation
                crop = cv2.warpPerspective(raw_image, M, (int(rectW), int(rectH)))

                # crop for 400 pixels height
                fHeight = (400 if rectH > 400 else rectH)
                crop = crop[0:int(fHeight), 0:int(rectW)]

                # Return the cropped strip and the respective angle
                return crop, angle

    return None, 0


def main():
    index = 0

    background_model = cv2.createBackgroundSubtractorKNN(history=500, detectShadows=True, dist2Threshold=150)
    trainBackground("/media/xtreme/data/XtremeCork/captures/captures/Background", background_model)

    image_data_X, image_data_Y = create_test_data("/media/xtreme/data/XtremeCork/captures/BagAcquisition_26072019")

    while True:
        target_FPS = 60
        start_time = time.time()  # Benchmarking time start
        if index >= len(image_data_X):
            pass
        img_original = cv2.imread(image_data_X[index])
        if (sum(img_original[-1, :, 0]) + sum(img_original[-1, :, 1]) + sum(img_original[-1, :, 2])) \
                / (len(img_original[-1, :, 0]) * 3) <= 1:
            index += 1
            continue
        else:
            index += 1

        processedIMG, angle = detectCorkStrip(img_original, background_model)

        if processedIMG is not None:
            target_FPS = 5
            cv2.imshow("Cork Detection", processedIMG)
        cv2.imshow("Original", img_original)

        elapsed_time_ms = (time.time() - start_time) * 1000
        wait_time_ms = int(1000 / target_FPS - elapsed_time_ms)
        wait_time_ms = 1 if wait_time_ms < 1 else wait_time_ms  # force a minimum wait of 1 ms
        k = cv2.waitKey(wait_time_ms)
        if k == 171:  # skip 25 frames
            index += 25
        if k == 173:  # back 25 frames
            index -= 25
        if k == 32:  # Use spacebar to pause the capture
            k = 0
            while k != 32:
                k = cv2.waitKey(1)
        if k == 27:  # exit on ESC key
            break

    pass


if __name__ == "__main__":
    main()
