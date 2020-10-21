# -*- coding: utf-8 -*-
import os
import logging
import PySpin
import cv2
import numpy as np
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session
from tensorflow.python.keras.layers import Dense, Activation, Flatten, Conv2D, MaxPooling2D, Dropout, BatchNormalization
from tensorflow.python.keras.models import Sequential
import time
import argparse
import matplotlib.pyplot as plt
import collections
import matplotlib
from collections import Counter

matplotlib.use('TkAgg')

# Directories
# DATADIR = "/media/xtreme/data/XtremeCork/captures/captures_1806"
TESTDIR = "/media/xtreme/data/XtremeCork/captures"
MODELDIR = "/home/xtreme/Documents/xtremecork-vision/src/resources/models"
BACKGROUND = "/media/xtreme/data/XtremeCork/captures/captures/Background"
TESTBAG = "BagAcquisition_26072019"             # BagAcquisition_26072019 // BagAcquisition_11072019
TESTDIR = os.path.join(TESTDIR, TESTBAG)

# CNNs
# MAIN_CNN = "cork_class_weights_new.h5"
# SIDE_CNN = "conv-32-32-64-128-dense-512-best.h5"
CNN = os.path.join("/home/xtreme/Documents/xtremecork-vision/src/resources/BEST/best_28-10-2019-accuracy100",
                   "conv-32-drop25-dense-128-drop50-64-drop50-1572285048.h5")
# CNN = os.path.join(MODELDIR, "2019-10-16", "conv-64-64(d25)-128(d25)-dense-512(d5)-1571240267.h5")

# Categories
CATEGORIES = ["Belly", "Back", "Side"]
CATEGORIES_NEW = ["Back", "Belly", "SideBellyLeft", "SideBellyRight"]
SIDE_CATEGORIES = ["BellyLeft", "BellyRight"]
TEST_CATEGORIES = ["Back", "Belly", "SideBellyUp", "SideBellyDown", "SideBellyLeft", "SideBellyRight"]

# Logger initialization
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
fh = logging.FileHandler('logging.txt')
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)
logger.addHandler(fh)
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
ch.setFormatter(formatter)
logger.addHandler(ch)


def loadCNN(file):
    """load .h5 model file

    :param file: path to the file
    :return: keras pre-trained model
    """
    return tf.keras.models.load_model(file)


def loadNN_main():
    # (50, 200)
    model = Sequential()
    # model.add(Conv2D(128, (15, 15), input_shape = X.shape[1:]))
    model.add(Conv2D(64, (1, 1), input_shape=(200, 50, 3)))
    # model.add(BatchNormalization())
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(128, (1, 1)))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(256, (1, 1)))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Dropout(0.5))

    model.add(Flatten())

    model.add(Dense(256))
    model.add(Activation("sigmoid"))

    model.add(Dense(3))
    model.add(Activation("softmax"))

    model.compile(loss="sparse_categorical_crossentropy",
                  optimizer="adam",
                  metrics=['accuracy'])

    f = os.path.join(MODELDIR, "cork_class_weights_new.h5")
    model.load_weights(f)

    return model


def prepareIMG(img):
    """Prepare image for CNN classification

    :param img: image to classify
    :return: processed image
    """
    IMG_SIZE = (50, 200)
    # IMG_SIZE = (100, 400)
    img = cv2.resize(img, IMG_SIZE)

    img = (img[..., ::-1].astype(np.float32)) / 255.0
    # img = cv2.normalize(img, None, alpha=0, beta=1,
    #                     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    img = np.expand_dims(img, axis=0)
    return img


def initCamera(cam):
    # cam parameters TODO: Make a config file
    CAM_exposure_time = 5000.0
    CAM_gain = 23.95
    CAM_WB_RED = 1.4
    CAM_WB_BLUE = 2.6

    # Initialize camera
    cam.Init()
    fps = PySpin.CFloatPtr(cam.GetNodeMap().GetNode("AcquisitionFrameRate")).GetValue()
    # print("Frame rate to be set to ", fps)
    logger.info(f"Frame rate to be set to  {fps}")

    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)

    cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)

    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
    cam.ExposureTime.SetValue(min(cam.ExposureTime.GetMax(), CAM_exposure_time))

    cam.GainAuto.SetValue(PySpin.GainAuto_Off)
    cam.Gain.SetValue(cam.Gain.GetMax())

    cam.BalanceWhiteAuto.SetValue(PySpin.BalanceWhiteAuto_Off)
    cam.BalanceRatioSelector.SetValue(PySpin.BalanceRatioSelector_Red)
    cam.BalanceRatio.SetValue(CAM_WB_RED)
    cam.BalanceRatioSelector.SetValue(PySpin.BalanceRatioSelector_Blue)
    cam.BalanceRatio.SetValue(CAM_WB_BLUE)

    # s_node_map = cam.GetTLStreamNodeMap()
    # buffer_count = PySpin.CIntegerPtr(s_node_map.GetNode('StreamBufferCountManual'))
    # buffer_count.SetValue(1)
    # logging.info(f"Buffer count now set to: {buffer_count.GetValue()}")

    logger.info("Finished setting up camera parameters")

    cam.BeginAcquisition()
    logger.info("Begin Acquisition")


def trainBackground(backgroundPath, fgbg):
    """Train Background using pre-existing Dataset

    :param backgroundPath: Path of the dataset
    :param fgbg: Background extraction model
    :return: Trained background model
    """
    for img in sorted(os.listdir(backgroundPath)):
        img_original = cv2.imread(os.path.join(backgroundPath, img))

        # remove lighting
        offset = img_original.shape[1] - 654
        img_cropped = img_original[:, int((offset / 2)):int((img_original.shape[1] - offset / 2))]

        # NOISE REDUCTION
        img_array = cv2.GaussianBlur(img_cropped, (7, 7), 0)

        fgmask = fgbg.apply(img_array)
    return fgmask


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
            if (img_cropped.shape[1] / 2 + img_cropped.shape[1] * 0.15) > center[0] > \
                    (img_cropped.shape[1] / 2 - img_cropped.shape[1] * 0.15):
                # Normalize the angles (0º is vertical)

                if rectW < rectH:
                    if angle < 0:
                        angle += 180
                else:
                    angle += 90

                # rows, cols = img_cropped.shape[0], img_cropped.shape[1]
                # M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
                #
                # diagonal = int(math.sqrt(math.pow(rows, 2) + math.pow(cols, 2)))
                #
                # # draw the outer contour (in green)
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                # cv2.drawContours(corkMask, [box], 0, 255, -1)
                # cv2.drawContours(img_cropped_rect, [box], 0, (0, 0, 255), 2)
                #
                # res = cv2.bitwise_and(img_cropped, img_cropped, mask=corkMask)
                #
                # img_rot = cv2.warpAffine(res, M, (diagonal, diagonal))
                # corkMask = cv2.warpAffine(corkMask, M, (diagonal, diagonal))
                #
                # contours, hierarchy = cv2.findContours(corkMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # x, y, w, h = cv2.boundingRect(contours[0])
                #
                # crop = img_rot[y:y + h, x:x + w]

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                src_pts = box.astype("float32")
                # count = 1
                # for point in src_pts:
                #     cv2.putText(img_cropped, f"{count}", (point[0], point[1]),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                #     count += 1

                if rectW < rectH:
                    dst_pts = np.array([[rectW - 1, 0],
                                        [rectW - 1, rectH - 1],
                                        [0, rectH - 1],
                                        [0, 0]], dtype="float32")
                    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
                    crop = cv2.warpPerspective(img_cropped, M, (int(rectW), int(rectH)))
                else:
                    dst_pts = np.array([[rectH - 1, rectW - 1],
                                        [0, rectW - 1],
                                        [0, 0],
                                        [rectH - 1, 0]], dtype="float32")
                    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
                    crop = cv2.warpPerspective(img_cropped, M, (int(rectH), int(rectW)))

                # Return the cropped strip and the respective angle
                return crop, angle

    return None, 0


def classify(img, angle, main_model, side_model):
    """ Classify the image using the main model and the side model

    :param img: Image to classify
    :param angle: Real angle of the cork strip
    :param main_model: Model for the main classification (Back, Belly, Side)
    :param side_model: Model for the side classification (Belly is Left,Right,Up,Down)
    :return: Classification of both models ((main_category, main_accuracy), (side_category, side_accuracy))
    """
    prediction_main = main_model.predict(img)
    prediction_main_accuracy = round(float(np.amax(prediction_main)) * 100, 2)
    prediction_main_category = CATEGORIES[np.argmax(prediction_main)]

    classification_main = (prediction_main_category, prediction_main_accuracy)

    prediction_side_category = None
    prediction_side_accuracy = 0

    if prediction_main_category == "Side":
        prediction_side = side_model.predict(img)
        prediction_side_category = SIDE_CATEGORIES[int(np.round(prediction_side))]
        if int(np.round(prediction_side)):  # BELLY RIGHT
            if 45 < angle < 135:
                prediction_side_category = "BellyDown"
            if angle > 135:
                prediction_side_category = "BellyLeft"

        else:  # BELLY LEFT
            if 45 < angle < 135:
                prediction_side_category = "BellyUp"
            if angle >= 135:
                prediction_side_category = "BellyRight"

        prediction_side_accuracy = 100 - round(float(prediction_side) * 100, 2) if float(prediction_side) < 0.5 \
            else round(float(prediction_side) * 100, 2)

    classification_side = (prediction_side_category, prediction_side_accuracy)

    return classification_main, classification_side


def classify_new(img, angle, model):
    """ Classify the image using the main model and the side model

    :param img: Image to classify
    :param angle: Real angle of the cork strip
    :param model: Model for the classification (Back, Belly, SideBellyRight, SideBellyLeft)
    :return: Classification of the model (category, accuracy)
    """
    prediction = model.predict(img)
    prediction_accuracy = round(float(np.amax(prediction)) * 100, 2)
    prediction_category = CATEGORIES_NEW[np.argmax(prediction)]

    # print(prediction_accuracy)

    if prediction_category == "SideBellyRight":
        if 45 < angle < 135:
            prediction_category = "SideBellyDown"
        if angle > 135:
            prediction_category = "SideBellyLeft"
    if prediction_category == "SideBellyLeft":
        if 45 < angle < 135:
            prediction_category = "SideBellyUp"
        if angle >= 135:
            prediction_category = "SideBellyRight"
    classification = (prediction_category, prediction_accuracy)
    return classification


def create_test_data(TESTDIR):
    x_test = []
    y_test = []

    for category in TEST_CATEGORIES:
        class_num = TEST_CATEGORIES.index(category)

        path = os.path.join(TESTDIR, category)
        try:
            listPath = os.listdir(path)
        except Exception:
            continue

        for img in sorted(listPath):
            x_test.append(os.path.join(path, img))
            y_test.append(class_num)

    return x_test, y_test


def main(live, gpu, noshow, benchmark):
    if gpu:
        # initialize gpu components
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
        config.log_device_placement = True  # to log device placement (on which device the operation ran)
        sess = tf.Session(config=config)
        set_session(sess)  # set this TensorFlow session as the default session for Keras
        logger.info("Using GPU")
    else:
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        logger.info("Using CPU")

    # Load both CNN for classification
    # main_model = loadNN_main()
    # side_model = loadCNN(os.path.join(MODELDIR, SIDE_CNN))
    model = loadCNN(CNN)

    background_model = cv2.createBackgroundSubtractorKNN(history=300, detectShadows=True, dist2Threshold=150)
    index = 0
    max_elapsed_time = 0
    benchmark_post_delay = 0
    failed_data = []
    start_classification = False

    if live:
        # cam parameters TODO: Make a config file
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

    else:
        # Background Subtraction
        trainBackground(BACKGROUND, background_model)
        logger.info("Training Background Subtraction")

        # load images
        image_data_X, image_data_Y = create_test_data(TESTDIR)
        logger.info(f"Loaded {len(image_data_X)} images from {TESTDIR}")

    failedCount = 0  # Evaluation count
    compared = 0  # Evaluation total

    SIZE_BENCH_TIME = 100
    SIZE_BENCH_CONF_M = 20

    benchmark_time = collections.deque([0.0] * SIZE_BENCH_TIME, maxlen=SIZE_BENCH_TIME)
    benchmark_confidence_main = collections.deque([0.0] * SIZE_BENCH_CONF_M, maxlen=SIZE_BENCH_CONF_M)
    benchmark_confidence_side = collections.deque([0.0] * SIZE_BENCH_CONF_M, maxlen=SIZE_BENCH_CONF_M)
    average_fps = collections.deque([0.0] * 10, maxlen=10)
    classification_counter = []

    fig = plt.figure(1, figsize=(1.66, 1.25))
    line_time, = plt.plot([0, ] * SIZE_BENCH_TIME, 'r-', lw=1)  # so that we can update data later
    plt.axis('off')
    plt.ylim(0, 100)

    fig2 = plt.figure(2, figsize=(1.66, 1.25))
    line_main_confidence, = plt.plot([0, ] * SIZE_BENCH_CONF_M, 'r-', lw=1)  # so that we can update data later
    plt.axis('off')
    plt.ylim(90, 102)

    fig3 = plt.figure(3, figsize=(1.66, 1.25))
    line_side_confidence, = plt.plot([0, ] * SIZE_BENCH_CONF_M, 'r-', lw=1)  # so that we can update data later
    plt.axis('off')
    plt.ylim(90, 102)
    while True:
        start_time = time.time()  # Benchmarking time start
        offlineClassifier = ""
        classification_final = ""
        target_FPS = 30  # Default target FPS for offline view
        wait_time_ms = 1
        if live:
            img = cam.GetNextImage()
            # handle incomplete images
            if img.GetImageStatus() != PySpin.IMAGE_NO_ERROR:
                logger.warning(f"Image incomplete with image status {img.GetImageStatus()}.")
                continue
            img_conv = img.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
            img_original = img_conv.GetNDArray()
            img.Release()
            # TODO: MUST OPTIMIZE - 5ms
            # for row in reversed(img_original):
            #     if sum(row[0]) + sum(row[1]) + sum(row[2]) == 0:
            #         badImage = True
            #         break
            # if badImage:
            #     logging.warning("Bad Image. Skipping...")
            #     continue
            if (sum(img_original[-1, :, 0]) + sum(img_original[-1, :, 1]) + sum(img_original[-1, :, 2])) \
                    / (len(img_original[-1, :, 0]) * 3) <= 1:
                logger.warning("Bad Image. Skipping...")
                continue

        else:
            if index >= len(image_data_X):
                # index = 0                                 # restart Dataset
                break
            img_original = cv2.imread(image_data_X[index])
            if (sum(img_original[-1, :, 0]) + sum(img_original[-1, :, 1]) + sum(img_original[-1, :, 2])) \
                    / (len(img_original[-1, :, 0]) * 3) <= 1:
                logger.warning("Bad Image. Skipping...")
                index += 1
                continue
            else:
                index += 1
        output = img_original.copy()

        ''' ---------------------------------------Classification--------------------------------------------------- '''
        # TODO: MUST OPTIMIZE - 4-8ms || detected 35+ ms (apply backgroundSubtraction)
        processedIMG, angle = detectCorkStrip(img_original, background_model)
        # Detected cork strip -> Start Classification process
        if processedIMG is not None:
            target_FPS = 5
            start_classification = True
            if not noshow:
                cv2.imshow("Cork Detection", processedIMG)
            processedIMG = prepareIMG(processedIMG)  # 0.4ms

            # classification_main, classification_side = classify(processedIMG, angle, main_model, side_model)  # 4ms GPU
            classification = classify_new(processedIMG, angle, model)  # 4ms GPU
            # print(f"class: {classification}")
            # add prediction percentages to benchmarking
            # benchmark_confidence_main.append(classification_main[1])
            # benchmark_confidence_side.append(classification_side[1])

            # cv2.putText(output, f"{classification_main[0]} ({classification_main[1]}%)", (20, 40),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(output, f"{classification[0]} ({classification[1]}%)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(output, f"angle: {int(np.round(angle))}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # offlineClassifier = classification_main[0]  # update the offline classifier
            offlineClassifier = (classification, angle)   # update the offline classifier

            # if classification_side[0] is not None:
            #     cv2.putText(output, f"{classification_side[0]} ({classification_side[1]}%)", (200, 40),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            #     offlineClassifier += classification_side[0]  # update the offline classifier
            classification_counter.append(offlineClassifier)
        # Not seeing any cork strip
        else:
            # End Classification process
            if start_classification:
                classification_all = Counter(classification_counter)
                # print(f"Classification: {classification_all}")
                classification_final = classification_all.most_common(1)[0][0]
                start_classification = False
                classification_counter = []

        ''' -------------------------------------------------------------------------------------------------------- '''
        if noshow:
            if offlineClassifier:
                # print("{} | {}".format(TEST_CATEGORIES[image_data_Y[index]], offlineClassifier))
                if not TEST_CATEGORIES[image_data_Y[index]] == offlineClassifier[0][0]:
                    failedCount += 1
                    failed_data.append((image_data_X[index], offlineClassifier, TEST_CATEGORIES[image_data_Y[index]]))
                compared += 1
            # if classification_final:
            #     if not TEST_CATEGORIES[image_data_Y[index - 1]] == classification_final:
            #         failedCount += 1
            #         failed_data.append((image_data_X[index - 1], classification_final))
            #     compared += 1

            # TODO: Save the failed classifications for future analysis
            print(f"Progress {index / len(image_data_X) * 100.0:.2f}%", end='\r')
        else:
            elapsed_time_ms = (time.time() - start_time) * 1000
            elapsed_time_ms += benchmark_post_delay
            benchmark_time.append(elapsed_time_ms)
            logger.debug(f"Software Execution Time: {elapsed_time_ms}ms")
            max_elapsed_time = elapsed_time_ms if elapsed_time_ms > max_elapsed_time else max_elapsed_time

            wait_time_ms = int(1000 / target_FPS - elapsed_time_ms)
            wait_time_ms = 1 if wait_time_ms < 1 else wait_time_ms  # force a minimum wait of 1 ms
            fps = int(1 / ((wait_time_ms + elapsed_time_ms) * 0.001)) if not live else int(
                1 / ((1 + elapsed_time_ms) * 0.001))
            average_fps.append(fps)
            cv2.putText(output, f"{round(sum(average_fps) / len(average_fps))} FPS", (700, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Classification", output)
        benchmark_post_delay = time.time()
        if benchmark:  # 20ms
            line_time.set_ydata(benchmark_time)
            line_main_confidence.set_ydata(benchmark_confidence_main)
            line_side_confidence.set_ydata(benchmark_confidence_side)

            '''--------------------------------------------------------------------------------------------------
                PROCESSING TIME BENCHMARK 
            '''
            plt.figure(1)
            ann = plt.annotate(f"{int(benchmark_time[-1])}", xy=(1, benchmark_time[-1]),
                               xytext=(0, 0), size=8,
                               color=line_time.get_color(), xycoords=('axes fraction', 'data'),
                               textcoords='offset points')
            fig.canvas.draw()
            ann.remove()
            overlay = output.copy()
            # convert canvas to image
            img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            # img is rgb, convert to opencv's default bgr
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # img = cv2.resize(img, None, fx=0.25, fy=0.25)

            h, w, c = img.shape  # of the small insert image
            x, y = (20, 340)
            overlay[y:y + h, x:x + w] = img  # that's it already ;)
            cv2.putText(overlay, "50", (x + 5, y + 65),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.putText(overlay, "Processing Time (ms)", (x + 30, y + 10),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.addWeighted(output, 0.7, overlay, 0.3, 0, output)
            '''--------------------------------------------------------------------------------------------------
                MAIN PREDICTION BENCHMARK 
            '''
            plt.figure(2)
            ann = plt.annotate(f"{benchmark_confidence_main[-1]:0.1f}", xy=(1, benchmark_confidence_main[-1]),
                               xytext=(-2, 0), size=6,
                               color=line_time.get_color(), xycoords=('axes fraction', 'data'),
                               textcoords='offset points')
            fig2.canvas.draw()
            ann.remove()
            overlay = output.copy()

            img = np.fromstring(fig2.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            img = img.reshape(fig2.canvas.get_width_height()[::-1] + (3,))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # img = cv2.resize(img, None, fx=0.25, fy=0.25)

            h, w, c = img.shape  # of the small insert image
            x, y = (20, 470)
            overlay[y:y + h, x:x + w] = img  # that's it already ;)
            cv2.putText(overlay, "95", (x + 5, y + 65),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.putText(overlay, "Main Confidence (%)", (x + 30, y + 10),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.addWeighted(output, 0.7, overlay, 0.3, 0, output)
            '''--------------------------------------------------------------------------------------------------
                SIDE PREDICTION BENCHMARK 
            '''
            plt.figure(3)
            ann = plt.annotate(f"{benchmark_confidence_side[-1]:0.1f}", xy=(1, benchmark_confidence_side[-1]),
                               xytext=(-2, 0), size=6,
                               color=line_time.get_color(), xycoords=('axes fraction', 'data'),
                               textcoords='offset points')
            fig3.canvas.draw()
            ann.remove()
            overlay = output.copy()

            img = np.fromstring(fig3.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            img = img.reshape(fig3.canvas.get_width_height()[::-1] + (3,))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # img = cv2.resize(img, None, fx=0.25, fy=0.25)

            h, w, c = img.shape  # of the small insert image
            x, y = (190, 470)
            overlay[y:y + h, x:x + w] = img  # that's it already ;)
            cv2.putText(overlay, "95", (x + 5, y + 65),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.putText(overlay, "Side Confidence (%)", (x + 30, y + 10),
                        cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 255), 1)
            cv2.addWeighted(output, 0.7, overlay, 0.3, 0, output)

            cv2.imshow("Classification", output)
        if live:
            k = cv2.waitKey(1)
            if k == 32:  # Use spacebar to pause the capture
                k = 0
                while k != 32:
                    k = cv2.waitKey(1)
                    cam.GetNextImage()
            if k == 27:  # exit on ESC key
                break
        else:
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
        # print(f"post benchmark delay: {(time.time() - benchmark_post_delay)*1000}")
        benchmark_post_delay = (time.time() - benchmark_post_delay) * 1000

    if noshow:
        logger.info(f"Finished evaluating! Failed {failedCount} times in {compared} images, "
                    f"overall accuracy of {(1 - (failedCount / compared)) * 100.0:.2f}.")
        for img, classification, real_class in failed_data:
            img_original = cv2.imread(img)

            cv2.putText(img_original, f"{classification[0][0]} ({classification[0][1]}%)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(img_original, f"angle: {int(np.round(classification[1]))}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.putText(img_original, f"{real_class}", (300, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Classification", img_original)
            cv2.waitKey(0)

    logger.info(f"Maximum Software delay was {max_elapsed_time} ms")

    # Clear camera parameters
    if live:
        cam.EndAcquisition()
        cam.DeInit()
        del cam


if __name__ == "__main__":
    logging.basicConfig(filename='log_filename.txt',
                        level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s')
    parser = argparse.ArgumentParser(description='Detection and classification of cork strips')
    parser.add_argument('--live', action='store_true', help='Use live camera capture')
    parser.add_argument('--gpu', action='store_true', help='Use GPU')
    parser.add_argument('--noshow', action='store_true', help='Hide the img display. For Offline use only.')
    parser.add_argument('--benchmark', action='store_true', help='Enable benchmarks')
    args = parser.parse_args()
    main(args.live, args.gpu, args.noshow, args.benchmark)
