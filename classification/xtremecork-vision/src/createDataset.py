import cv2
import os
import logging
import shutil
import pathlib
import datetime
import random
# from evaluateCNN import trainBackground
# from croppingTest import detectCorkStrip
#
# Directories
# BACKGROUND = "/media/xtreme/data/XtremeCork/captures/captures/Background"  # background subtraction train set
# DATA_SRC_DIR = "/media/xtreme/data/XtremeCork/captures"  # path to the raw captures
# DATASET_DEST_DIR = "/media/xtreme/data/XtremeCork/datasets/dataset_jan_2020"  # destination for the created dataset
# CORK_DIR = "/media/xtreme/data/XtremeCork/datasets/corks_jan_2020"
# DATA_CAT = ["Back", "Belly", "SideBellyDown", "SideBellyUp"]  # labels from raw captures
# SET_CAT = ["Back", "Belly", "SideBellyLeft", "SideBellyRight"]  # labels for dataset
# ACQUISITIONS = ["BagAcquisition_26072019", "BagAcquisition_11072019"]

CORK_DIR = "img/dataset_divided"
DATA_SRC_DIR = "/img/dataset_divided"  # path to the raw captures
DATASET_DEST_DIR = "img/processed"  # path to the raw captures
DATA_CAT = ["back", "Belly", "SideBellyDown", "SideBellyUp"]  # labels from raw captures
SET_CAT = ["back", "belly", "belly_left", "belly_right"]  # labels for dataset


# Logger initialization
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
fh = logging.FileHandler(os.path.join('logs', 'application_log', f'log_createDataset-{datetime.date.today()}.txt'))
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)
logger.addHandler(fh)
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
ch.setFormatter(formatter)
logger.addHandler(ch)


def extract_cork_strips():
    # train the background subtractor
    # background_model = cv2.createBackgroundSubtractorKNN(history=300, detectShadows=True, dist2Threshold=80)
    # trainBackground(BACKGROUND, background_model)
    for captures in ACQUISITIONS:
        for category in SET_CAT:
            # TODO: Handle bad detections
            # Check if Acquistion set was already processed
            # if os.path.exists(os.path.join(CORK_DIR, 'AcquisitionList.txt')):
            #     with open(os.path.join(CORK_DIR, 'AcquisitionList.txt')) as f:
            #         fLines = f.readlines()
            #     AcquisitionList = [x.strip() for x in fLines]
            #     if captures in AcquisitionList:
            #         logger.info(f"Acquisition set {captures} is already processed!")
            #         break

            path_new = os.path.join(DATA_SRC_DIR, captures, category)
            listPath = [os.path.join(path_new, f) for f in os.listdir(path_new)]

            # if category == "SideBellyDown": category = "SideBellyRight"
            # if category == "SideBellyUp": category = "SideBellyLeft"

            destination_category = category
            counter = 0
            cork_count = 1
            detection_count = 0
            destination_directory = CORK_DIR

            if os.path.exists(os.path.join(CORK_DIR, category)):
                if len(os.listdir(os.path.join(CORK_DIR, category))) != 0:
                    listCork = sorted(os.listdir(os.path.join(CORK_DIR, category)), key=int)
                    print(listCork)
                    cork_count = int(listCork[-1]) + 1

            for img in sorted(listPath):
                # read image
                raw_image = cv2.imread(img)
                # processedIMG, angle = detectCorkStrip(raw_image, background_model)
                if processedIMG is None:
                    if detection_count > 5:
                        logger.info(
                            f"Saved cork strip {cork_count:04d} with {counter:02d} images to category {destination_category}.")
                        cork_count += 1
                        counter = 0
                    detection_count = 0
                else:
                    detection_count += 1

                    # create all necessary directories
                    destination_path = os.path.join(destination_directory, destination_category, f"{cork_count:04d}")
                    pathlib.Path(destination_path).mkdir(parents=True, exist_ok=True)

                    dest_img_path = os.path.join(destination_path, f"{destination_category}-{cork_count:04d}-{counter:02d}.jpg")
                    counter += 1
                    cv2.imwrite(dest_img_path, processedIMG)

            logger.info(f"Category {category}, with a total of {cork_count - 1} cork strips")
        with open(os.path.join(CORK_DIR, 'AcquisitionList.txt'), 'a') as f:
            f.write(f"{captures}\n")
            logger.info(f"Added capture {captures} to AcquisitionList.txt")


def main():
    # extract_cork_strips()

    # Separate into train // validation // testing datasets
    validation_split = 0.2      # % for validation set
    testing_split = 0.2         # % for testing set
    for category in SET_CAT:
        logger.info(f"Starting to move category {category}.")
        count = 1
        listCork = sorted(os.listdir(os.path.join(CORK_DIR, category)), key=int)
        total_num_strips = listCork[-1]
        random.shuffle(listCork)
        for cork in listCork:
            src_cork_dir = os.path.join(CORK_DIR, category, cork)
            dst_cork_dir = os.path.join(DATASET_DEST_DIR, "train", category)

            if count > int(int(total_num_strips) * (1 - validation_split - testing_split)):
                dst_cork_dir = os.path.join(DATASET_DEST_DIR, "valid", category)
            if count > int(int(total_num_strips) * (1 - testing_split)):
                dst_cork_dir = os.path.join(DATASET_DEST_DIR, "test", category)

            # create all necessary directories
            pathlib.Path(dst_cork_dir).mkdir(parents=True, exist_ok=True)
            # for img in os.listdir(src_cork_dir):
            shutil.copy(src_cork_dir, os.path.join(dst_cork_dir, cork + ".jpg"))

            count += 1
            logger.info(f"Copied cork {cork} from {category} to {dst_cork_dir}")
        train_count = int(int(total_num_strips) * (1 - validation_split - testing_split))
        valid_count = int(int(total_num_strips) * validation_split)
        logger.info(f"Final Split is "
                    f"TRAIN = {int(int(total_num_strips) * (1 - validation_split - testing_split))} "
                    f"VALID = {int(int(total_num_strips) * validation_split)} "
                    f"TEST  = {int(total_num_strips) - train_count - valid_count}")


if __name__ == "__main__":
    main()
