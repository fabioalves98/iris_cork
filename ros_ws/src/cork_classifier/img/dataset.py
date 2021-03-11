import os
import cv2
from shutil import copyfile


def createDataset():
    directories = [
        'old',
        'plane',
        'inclined'
    ]
    final_dir = 'dataset/'

    idx = 0

    for directory in directories:
        for filename in os.listdir(directory):
            if filename.startswith("belly_left"): 
                print(os.path.join(directory, filename))
                pfile = os.path.join(directory, filename)
                copyfile(pfile, final_dir + 'belly_left_' + str(idx) + '.jpg')
                idx += 1
                continue
            else:
                continue


def load_and_rotate(folder, filename_, new_name):

    file_id = 0
    for filename in os.listdir(folder):
        if filename.startswith(filename_):
            print("loaded ", filename)
            img = cv2.imread(os.path.join(folder, filename))
            if img is not None:
                rotated = cv2.rotate(img, cv2.ROTATE_180)
                cv2.imwrite(folder + new_name + str(file_id) + ".jpg", rotated)
                file_id += 1
    
    
   

if __name__ == '__main__':

    createDataset()

    # load_and_rotate("inclined/", "belly_right_", "belly_left_")