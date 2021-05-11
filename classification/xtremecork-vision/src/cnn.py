import argparse
import os
import pickle
import random
import time
import datetime
import pathlib
import cv2
import numpy as np
import pandas as pd
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session
from pandas import Series
# from sklearn.externals import joblib
import joblib
from tensorflow.python.keras.layers import Dense, Dropout, Activation, Flatten, Conv2D, MaxPooling2D, BatchNormalization
from tensorflow.python.keras.models import Sequential
from keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import TensorBoard, ModelCheckpoint
#
# from sklearn.externals import joblib

DIR = os.path.dirname(__file__)
RESDIR = os.path.join(DIR, "resources")
# GOOGLE COLAB
GOOGLEDATADIR = "/content/cork_dataset"
# LOCAL DIR
# TRAINDATADIR = os.path.join(os.sep, "media", "xtreme", "data", "XtremeCork", "datasets", "dataset_october", "train")
# VALIDDATADIR = os.path.join(os.sep, "media", "xtreme", "data", "XtremeCork", "datasets", "dataset_october", "valid")
# TESTDATADIR = os.path.join(os.sep, "media", "xtreme", "data", "XtremeCork", "datasets", "dataset_october", "test")
MODELDIR = os.path.join(DIR, "resources", "models", f"{datetime.date.today()}")
RESULTDIR = os.path.join(RESDIR, "results", f"{datetime.date.today()}")
LOGS = os.path.join(DIR, "logs", f"logs-{datetime.date.today()}")

# LOCAL DIR
# TRAINDATADIR = os.path.join("img", "processed", "train")
# VALIDDATADIR = os.path.join(DIR, "img", "processed", "valid")
# TESTDATADIR = os.path.join(DIR, "img", "processed", "test")
TRAINDATADIR = os.path.join(os.getcwd(), "img/processed/train/")
VALIDDATADIR = "img/processed/valid/"
TESTDATADIR = "img/processed/test/"

# MODELDIR = os.path.join(DIR, "resources", "models", f"{datetime.date.today()}")
MODELDIR = os.path.join(DIR, "resources", "models", "2021-05-04")
RESULTDIR = os.path.join(RESDIR, "results", f"{datetime.date.today()}")
LOGS = os.path.join(DIR, "logs", f"logs-{datetime.date.today()}")

# Create the necessary directories
pathlib.Path(MODELDIR).mkdir(parents=True, exist_ok=True)
pathlib.Path(RESULTDIR).mkdir(parents=True, exist_ok=True)
pathlib.Path(LOGS).mkdir(parents=True, exist_ok=True)

CATEGORIES = ["back", "belly", "belly_left", "belly_right"]
# SIDE_CATEGORIES = ["SideBellyLeft", "SideBellyDown", "SideBellyUp", "SideBellyRight"]

# IMG_SIZE = (400, 100)
IMG_SIZE = (200, 50)
NAME = ""


# def create_training_data():
#     training_data = []
#     test_data = []
#
#     # training and test dataset
#     x_train = []
#     x_test = []
#     y_train = []
#     y_test = []
#     # FILL TRAIN DATA
#     for category in CATEGORIES:
#         class_num = CATEGORIES.index(category)
#         path = os.path.join(TRAINDATADIR, category)
#         listPath = os.listdir(path)
#         random.shuffle(listPath)
#         print(f"Searching {path} and found {len(listPath)} images")
#         for img in listPath:
#             try:
#                 img_array = cv2.imread(os.path.join(path, img))
#                 img_array = cv2.normalize(img_array, None, alpha=0, beta=1,
#                                           norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
#                 new_array = cv2.resize(img_array, IMG_SIZE)
#                 training_data.append([new_array, class_num])
#             except Exception as e:
#                 pass
#
#     # FILL TEST DATA
#     for category in CATEGORIES:
#         class_num = CATEGORIES.index(category)
#         path = os.path.join(TESTDATADIR, category)
#         listPath = os.listdir(path)
#         random.shuffle(listPath)
#         print(f"Searching {path} and found {len(listPath)} images")
#         for img in listPath:
#             try:
#                 img_array = cv2.imread(os.path.join(path, img))
#                 img_array = cv2.normalize(img_array, None, alpha=0, beta=1,
#                                           norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
#                 new_array = cv2.resize(img_array, IMG_SIZE)
#                 test_data.append([new_array, class_num])
#             except Exception as e:
#                 pass
#
#     print(f"train size: {len(training_data)}")
#     print(f"test size: {len(test_data)}")
#
#     random.shuffle(training_data)
#     random.shuffle(test_data)
#
#     for features, label in training_data:
#         x_train.append(features)
#         y_train.append(label)
#
#     for features, label in test_data:
#         x_test.append(features)
#         y_test.append(label)
#
#     x_train = np.array(x_train).reshape(-1, IMG_SIZE[1], IMG_SIZE[0], 3)
#     x_test = np.array(x_test).reshape(-1, IMG_SIZE[1], IMG_SIZE[0], 3)
#
#     # print('Min: %.3f, Max: %.3f' % (x_train.min(), x_train.max()))
#
#     return x_train, y_train, x_test, y_test


def saveTrainingSet(x_train, y_train, x_test, y_test):
    filename = os.path.join(RESDIR, 'x_train.sav')
    joblib.dump(x_train, filename)

    filename = os.path.join(RESDIR, 'y_train.sav')
    joblib.dump(y_train, filename)

    filename = os.path.join(RESDIR, 'x_test.sav')
    joblib.dump(x_test, filename)

    filename = os.path.join(RESDIR, 'y_test.sav')
    joblib.dump(y_test, filename)


def loadTrainingSet():
    return joblib.load(os.path.join(RESDIR, 'x_train.sav')), \
           joblib.load(os.path.join(RESDIR, 'y_train.sav')), \
           joblib.load(os.path.join(RESDIR, 'x_test.sav')), \
           joblib.load(os.path.join(RESDIR, 'y_test.sav'))


# def trainModel(X, y, testX, testY, e=1, val=0.3):
def trainModel(epoch=1):
    global NAME
    NAME += "conv-64+64(d0)"
    model = Sequential()
    # model.add(Conv2D(128, (15, 15), input_shape = X.shape[1:]))
    model.add(Conv2D(64, (3, 3), input_shape=(IMG_SIZE[0], IMG_SIZE[1], 1)))
    # model.add(BatchNormalization())
    model.add(Activation("relu"))
    model.add(Conv2D(64, (3, 3)))
    model.add(Activation("relu"))
    # model.add(Dropout(rate=0.25))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    NAME += "-128+128(d0)"
    model.add(Conv2D(128, (3, 3)))
    model.add(Activation("relu"))
    model.add(Conv2D(128, (3, 3)))
    model.add(Activation("relu"))
    # model.add(Dropout(rate=0.25))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    NAME += "-256+256+256(d0)"
    model.add(Conv2D(256, (3, 3)))
    model.add(Activation("relu"))
    model.add(Conv2D(256, (3, 3)))
    model.add(Activation("relu"))
    model.add(Conv2D(256, (3, 3)))
    model.add(Activation("relu"))
    # model.add(Dropout(rate=0.25))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Flatten())

    NAME += "-dense-1024(d0)"
    model.add(Dense(2048))
    model.add(Activation("relu"))
    # model.add(BatchNormalization())
    # model.add(Dropout(rate=0.5))
    NAME += "-dense-1024(d0)"
    model.add(Dense(1024))
    model.add(Activation("relu"))
    # model.add(BatchNormalization())
    # model.add(Dropout(rate=0.5))

    model.add(Dense(4))
    model.add(Activation("softmax"))

    NAME += f"-{int(time.time())}"
    model.compile(loss="categorical_crossentropy",
                  optimizer="adam",
                  metrics=['accuracy'])
    
    print("b4")
    print(TRAINDATADIR)

    # CALLBACKS
    save_best = ModelCheckpoint(os.path.join(MODELDIR, f"best-{NAME}.h5"), monitor='val_loss', mode='min',
                                save_best_only=True, verbose=1)
    tensorboard = TensorBoard(log_dir=os.path.join(LOGS, NAME))
    # model.fit(X, y,
    #           epochs=e,
    #           validation_split=val,
    #           batch_size=8,
    #           callbacks=[save_best])
    BS = 16

    train_datagen = ImageDataGenerator(
        rescale=1. / 255,
        zoom_range=[1.0, 1.5],
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.3,
        vertical_flip=True,
        fill_mode="nearest")

    valid_datagen = ImageDataGenerator(rescale=1. / 255)
    # test_datagen = ImageDataGenerator(rescale=1. / 255)
    train_generator = train_datagen.flow_from_directory(
        directory=TRAINDATADIR,
        target_size=IMG_SIZE,
        color_mode="grayscale",
        batch_size=BS,
        class_mode="categorical",
        shuffle=True,
        seed=42
    )

    valid_generator = valid_datagen.flow_from_directory(
        directory=VALIDDATADIR,
        target_size=IMG_SIZE,
        color_mode="grayscale",
        batch_size=1,
        class_mode="categorical",
        shuffle=True,
        seed=42
    )

    STEP_SIZE_TRAIN = train_generator.n // train_generator.batch_size
    STEP_SIZE_VALID = valid_generator.n // valid_generator.batch_size
    print(STEP_SIZE_TRAIN, STEP_SIZE_VALID,  train_generator.n,  valid_generator.n, valid_generator.batch_size, train_generator.batch_size)
    model.fit_generator(generator=train_generator,
                        steps_per_epoch=STEP_SIZE_TRAIN,
                        validation_data=valid_generator,
                        validation_steps=STEP_SIZE_VALID,
                        epochs=epoch,
                        callbacks=[save_best, tensorboard])

    print("after")
    model.save(os.path.join(MODELDIR, NAME + '.h5'))
    # model.evaluate_generator(generator=valid_generator,
    #                          steps=STEP_SIZE_VALID,
    #                          verbose=1)

    # results.to_csv("results.csv", index=False)
    # results.to_csv(os.path.join(RESULTDIR, f"results-{NAME}.csv"), index=False)

    # model.save_weights('cork_class_weights.h5')
    # print("Saved model as cork_classifier.model")

    # model = tf.keras.models.load_model(f"best-{NAME}.h5")
    # test_generator.reset()
    # pred = model.predict_generator(test_generator,
    #                                steps=STEP_SIZE_TEST,
    #                                verbose=1)
    #
    # export_to_excel(test_generator, pred)

    return model


def export_to_excel(NAME, test_generator, pred, suffix=""):
    # Save all information in a formatted excel file!
    predicted_class_indices = np.argmax(pred, axis=1)
    labels = test_generator.class_indices
    labels = dict((v, k) for k, v in labels.items())
    predictions = [labels[k] for k in predicted_class_indices]
    # full_predictions = [f"{round(p1,2):1.2f}|{round(p2,2):1.2f}|{round(p3,2):1.2f}|{round(p4,2):1.2f}"
    #                     for p1, p2, p3, p4 in (predi for predi in pred.tolist())]
    # full_predictions = [[p1, p2, p3, p4] for p1, p2, p3, p4 in (predi for predi in pred.tolist())]

    # get individual prediction accuracy for each cork side
    pred_back_pct = [round(row[0], 2)*100 for row in pred]
    pred_belly_pct = [round(row[1], 2) * 100 for row in pred]
    pred_sidebellyleft_pct = [round(row[2], 2) * 100 for row in pred]
    pred_sidebellyight_pct = [round(row[3], 2) * 100 for row in pred]

    filenames = test_generator.filenames
    filepaths = test_generator.filepaths
    correct_class = [k[0] for k in (x.split('/') for x in filenames)]
    cork_id = [int(k[1]) for k in (x.split('-') for x in filenames)]

    correct_guesses = [{}, {}, {}, {}]
    total_guesses = [{}, {}, {}, {}]
    maxmin_guesses = [{}, {}, {}, {}]

    correct_pred_counter = 0
    total_pred_counter = 0

    for idx in range(len(predictions)):
        category_index = CATEGORIES.index(correct_class[idx])
        cork_identification = cork_id[idx]

        # initialize the guess for cork idx
        if cork_identification not in correct_guesses[category_index].keys():
            correct_guesses[category_index][cork_identification] = 0
            total_guesses[category_index][cork_identification] = 0
            maxmin_guesses[category_index][cork_identification] = [100, 0]

        if pred[idx][category_index] < maxmin_guesses[category_index][cork_identification][0]:
            maxmin_guesses[category_index][cork_identification][0] = pred[idx][category_index]

        # count only if guess is correct
        if correct_class[idx] == predictions[idx]:
            # increment correct guess
            correct_guesses[category_index][cork_identification] = correct_guesses[category_index][
                                                                       cork_identification] + 1
            # if np.amax(pred[idx]) < maxmin_guesses[category_index][cork_identification][0]:
            #     maxmin_guesses[category_index][cork_identification][0] = np.amax(pred[idx])
            correct_pred_counter += 1
        else:
            if np.amax(pred[idx]) > maxmin_guesses[category_index][cork_identification][1]:
                maxmin_guesses[category_index][cork_identification][1] = np.amax(pred[idx])
        total_guesses[category_index][cork_identification] = total_guesses[category_index][cork_identification] + 1
        total_pred_counter += 1

    # prediction columns
    pred_results_id = ["ID"]
    pred_results_correct_guess = ["Correct Guesses"]
    pred_results_total_guess = ["Total Guesses"]
    pred_results_accuracy = ["Accuracy (%)"]
    pred_results_lowgood = ["Low Good"]
    pred_results_highbad = ["High Bad"]

    for label in range(len(CATEGORIES)):
        pred_results_id += ["Category"] + list(correct_guesses[label].keys())
        pred_results_correct_guess += [CATEGORIES[label]] + list(correct_guesses[label].values())
        pred_results_total_guess += [""] + list(total_guesses[label].values())
        pred_results_accuracy += [""] + [round((float(x) / float(y) * 100), 2) for x, y in
                                   zip(list(correct_guesses[label].values()), list(total_guesses[label].values()))]
        # pred_results_c5 += [""] + [f"WG: {round(maxBadGuess,2):1.2f} | BB: {round(minGoodGuess,2):1.2f}"
        #                            for maxBadGuess, minGoodGuess in (x for x in maxmin_guesses[label].values())]
        pred_results_lowgood += [""] + [round(row[0], 2)*100 for row in maxmin_guesses[label].values()]
        pred_results_highbad += [""] + [round(row[1], 2)*100 for row in maxmin_guesses[label].values()]

    d = dict(File=[f'=HYPERLINK("file://{f}", "{f.split("/")[-1]}")' for f in filepaths],
             Guess=predictions,
             Correct=correct_class,
             ID=cork_id,
             BAC=pred_back_pct,
             BEL=pred_belly_pct,
             SBL=pred_sidebellyleft_pct,
             SBR=pred_sidebellyight_pct,
             F=[],
             G=pred_results_id,
             H=pred_results_correct_guess,
             I=pred_results_total_guess,
             J=pred_results_accuracy,
             LG=pred_results_lowgood,
             HB=pred_results_highbad,
             L=["Overall %", round(correct_pred_counter / total_pred_counter * 100.0, 2)],
             )
    results = pd.DataFrame(dict([(k, Series(v)) for k, v in d.items()]))
    results.style.apply(style_compare, axis=None).to_excel(os.path.join(RESULTDIR, f"results-{NAME}-{suffix}.xlsx"),
                                                           engine='openpyxl')
#     results.style.format(make_clickable)
# def make_clickable(val):
#     return '<a href="{}">{}</a>'.format(val, val)

def style_compare(x):
    """
      Colors background elements in a dataframe
      green if equal and red if not.
    """
    bad = "red"
    good = "green"

    m1 = x["Guess"] == x["Correct"]
    m2 = x["Guess"] != x["Correct"]

    frame = pd.DataFrame('background-color: white', index=x.index, columns=x.columns)
    # rewrite values by boolean masks
    frame["Guess"] = np.where(m1, f'background-color: {good}', frame["Guess"])
    frame["Guess"] = np.where(m2, f'background-color: {bad}', frame["Guess"])

    return frame


def main(loadCNN):
    config = tf.compat.v1.ConfigProto()
    config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
    config.log_device_placement = True  # to log device placement (on which device the operation ran)
    sess = tf.compat.v1.Session(config=config)
    set_session(sess)  # set this TensorFlow session as the default session for Keras

    global NAME
    suffix = ""

    # x_train, y_train, x_test, y_test = create_training_data()
    # try:
    #     saveTrainingSet(x_train, y_train, x_test, y_test)
    # except Exception:
    #     pass
    # x_train, y_train, x_test, y_test = loadTrainingSet()

    # Normalize training and test datasets
    # x_train = tf.keras.utils.normalize(x_train, axis=1)
    # x_test = tf.keras.utils.normalize(x_test, axis=1)

    # TRAINING MODEL
    if not loadCNN:
        model = trainModel(50)
    else:
        # Search for CNN in models folder
        path_to_model = ""
        for root, dirs, files in os.walk(os.path.join(RESDIR, "models")):
            for file in files:
                if loadCNN in files:
                    path_to_model = os.path.join(root, loadCNN)
                    break
        try:
            model = tf.keras.models.load_model(path_to_model)
            print(f"Successfully loaded model {loadCNN}!")
            NAME = f"{loadCNN[:-3]}"
            suffix = "manual"
        except IOError as e:
            print("Did not load model correctly!")
            print(e)
            return

    print("Testing model.")

    test_datagen = ImageDataGenerator(rescale=1. / 255)
    test_generator = test_datagen.flow_from_directory(
        directory=TESTDATADIR,
        target_size=IMG_SIZE,
        color_mode="grayscale",
        batch_size=1,
        class_mode=None,
        shuffle=False,
        seed=42
    )
    STEP_SIZE_TEST = test_generator.n // test_generator.batch_size

    # Test the trained model
    test_generator.reset()
    pred = model.predict_generator(test_generator,
                                   steps=STEP_SIZE_TEST,
                                   verbose=1)

    print(np.matrix.round(pred, decimals=2))
    folders = ["back", "belly", "belly_left", "belly_right"]
    correct_labels = []
    for f in folders:
        path_joined = os.path.join(TESTDATADIR, f)
        for filename in os.listdir(path_joined):
            correct_labels.append(f)

    i = 0
    final_answer = []
    low_percent = []
    for row in pred:
        idx = np.argmax(row)
        idx_label = folders.index(correct_labels[i])
        max_val = np.amax(row)
        if max_val < 0.8:
            low_percent.append(1)
        else:
            low_percent.append(0)
        
        if idx == idx_label:
            final_answer.append(1)
        else:
            final_answer.append(0)
        i+=1
    print(final_answer)
    print("SUCCESS PERCENTAGE: ", np.count_nonzero(final_answer)/len(final_answer))
    print("LOW PERCENTAGE VALUE PERCENTAGE: ", np.count_nonzero(low_percent)/len(low_percent))


    # model.save(os.path.join(MODELDIR, NAME + '.h5'))
    # export_to_excel(NAME, test_generator, pred, suffix)

    # Load and test the best model
    # best_model = tf.keras.models.load_model(os.path.join(MODELDIR, f"best-{NAME}.h5"))
    # test_generator.reset()
    # best_pred = best_model.predict_generator(test_generator,
    #                                          steps=STEP_SIZE_TEST,
    #                                          verbose=1)
    # export_to_excel(test_generator, best_pred, "best-")
    # LOADING TRAINED MODEL
    # model = tf.keras.models.load_model(model_name)
    #
    # # print(x_train.shape[1:])
    # # print("Testing model.")
    #
    # # TEST MODEL
    # idx = 0
    # failed = 0
    # predList = []
    #
    # predictions = model.predict(x_test)
    # for pred in predictions:
    #     # print(pred)
    #     predList.append(np.argmax(pred))
    #
    # for p in predList:
    #     if p != y_test[idx]:
    #         failed += 1
    #         print(f"predicted {p} instead of {y_test[idx]}")
    #     idx += 1
    # print("failed {} in {} or {}%".format(failed, len(predictions), failed / len(predictions) * 100))


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Detection and classification of cork strips')
    parser.add_argument('-l', '-load', action='store', dest='cnn', help='Load model instead of training a new one.')
    args = parser.parse_args()
    main(args.cnn)
