import gc
import os
import pickle
import random
import datetime
import time
import pathlib
import cv2
import numpy as np
import pandas as pd
import tensorflow as tf

from sklearn.externals import joblib
from itertools import product, permutations
from keras import backend as K
from tensorflow.python.keras.layers import Dense, Dropout, Activation, Flatten, Conv2D, MaxPooling2D, BatchNormalization
from tensorflow.python.keras.models import Sequential
from keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import TensorBoard, ModelCheckpoint
from pandas import Series
from numba import cuda


#  ------------------------ DIRECTORIES and OTHER VARS ------------------------------
# DIR = os.path.dirname(__file__)
DIR = "/media/xtreme/data/XtremeCork/models_temp"
RESDIR = os.path.join(DIR, "resources")
# GOOGLE COLAB
GOOGLEDATADIR = "/content/captures_1806_crop"
# LOCAL DIR
# LOCALDATADIR = "/media/xtreme/data/XtremeCork/datasets/side_dataset"

TRAINDATADIR = "/media/xtreme/data/XtremeCork/datasets/dataset_october/train"
VALIDDATADIR = "/media/xtreme/data/XtremeCork/datasets/dataset_october/valid"
TESTDATADIR = "/media/xtreme/data/XtremeCork/datasets/dataset_october/test"

MODELDIR = os.path.join(RESDIR, "models", f"{datetime.date.today()}")
RESULTDIR = os.path.join(RESDIR, "results", f"{datetime.date.today()}")
LOGS = os.path.join(DIR, "logs", f"logs-{datetime.date.today()}")

# CATEGORIES = ["BellyLeft", "BellyRight"]
CATEGORIES = ["Back", "Belly", "SideBellyLeft", "SideBellyRight"]

IMG_SIZE = (200, 50)

# Create the necessary directories
pathlib.Path(MODELDIR).mkdir(parents=True, exist_ok=True)
pathlib.Path(RESULTDIR).mkdir(parents=True, exist_ok=True)
pathlib.Path(LOGS).mkdir(parents=True, exist_ok=True)

# model_name = os.path.join(MODELDIR, f'conv-32-64-128-256-dense-256-{time.time()}.h5')

# -------------------  LAYER VALUES ----------------------------
conv_filter_sizes = [32, 64, 128, 256]
dense_filter_sizes = [64, 128, 256]

conv_layers = [1, 2, 3]
dense_layers = [0, 1, 2]

'''
conv-32
conv-32-dense-128
conv-32-dense-256
conv-32-dense-512
conv-32-64
conv-32-64-dense-128
conv-32-64-dense-256
'''


# def loadTrainingSet():
#     return joblib.load('x_train.sav'), joblib.load('y_train.sav'), \
#            joblib.load('x_test.sav'), joblib.load('y_test.sav')

# def trainModel(X, y, x_test, y_test, e=1, val=0.2):
def combinations(number_filters, size_filters):
    l = []
    for num in number_filters:
        l += (list(permutations(size_filters, num)))
    return l

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

def trainModel(conv_list, dense_list, e=1):
    # Start Tensorflow GPU
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
    config.log_device_placement = True  # to log device placement (on which device the operation ran)

    # Visualize the models to be trained
    print("Starting training on the following models:\n\n")
    NAME = ""
    for conv in conv_list:
        for dense in dense_list:
            NAME = "conv-32"
            # NAME += "-drop25"
            for c_layer in conv:
                NAME += f"-{c_layer}"
                # NAME += "-drop25"
            NAME += "-dense"
            for d_layer in dense:
                if d_layer != 0:
                    NAME += f"-{d_layer}"
                    # NAME += "-drop25"
            print(NAME)
    key_input = input("Press Enter to Continue or (q)uit to exit.")
    if key_input == 'q' or key_input == 'Q': return

    counter = 1
    # create the model architectures
    for conv in conv_list:
        for dense in dense_list:
            sess = tf.Session(config=config)
            K.set_session(sess)                 # set this TensorFlow session as the default session for Keras

            print(f"CNN {counter} of {len(conv_list) * len(dense_list)}")
            firstlayer = True
            for c_layer in conv:
                if firstlayer:
                    NAME = f"conv-{c_layer}"
                    model = Sequential()
                    model.add(Conv2D(c_layer, (3, 3), input_shape=(IMG_SIZE[0], IMG_SIZE[1], 3)))
                    model.add(Activation("relu"))
                    NAME += "-drop25"
                    model.add(Dropout(0.25))
                    model.add(MaxPooling2D(pool_size=(2, 2)))
                    firstlayer = False
                else:
                    NAME += f"-{c_layer}"
                    model.add(Conv2D(c_layer, (3, 3)))
                    model.add(Activation("relu"))
                    NAME += "-drop25"
                    model.add(Dropout(0.25))
                    model.add(MaxPooling2D(pool_size=(2, 2)))
            model.add(Flatten())
            NAME += "-dense"
            for d_layer in dense:
                if d_layer != 0:
                    NAME += f"-{d_layer}"
                    model.add(Dense(d_layer))
                    model.add(Activation("relu"))
                    NAME += "-drop50"
                    model.add(Dropout(0.5))
            # Final Layer
            model.add(Dense(4))
            model.add(Activation("softmax"))
            # print(NAME)
            NAME += f"-{int(time.time())}"

            tensorboard = TensorBoard(log_dir=os.path.join(LOGS, NAME))

            model.compile(loss="categorical_crossentropy",
                          optimizer="adam",
                          metrics=['accuracy'])

            save_best = ModelCheckpoint(os.path.join(MODELDIR, f"{NAME}-best.h5"), monitor='val_loss', mode='min',
                                        save_best_only=True, verbose=1)

            BS = 16

            train_datagen = ImageDataGenerator(
                rescale=1. / 255,
                zoom_range=[1.0, 1.3],
                width_shift_range=0.1,
                height_shift_range=0.1,
                shear_range=0.15,
                vertical_flip=True,
                fill_mode="nearest")

            valid_datagen = ImageDataGenerator(rescale=1. / 255)
            test_datagen = ImageDataGenerator(rescale=1. / 255)

            train_generator = train_datagen.flow_from_directory(
                directory=TRAINDATADIR,
                target_size=IMG_SIZE,
                color_mode="rgb",
                batch_size=BS,
                class_mode="categorical",
                shuffle=True,
                seed=42
            )

            valid_generator = valid_datagen.flow_from_directory(
                directory=VALIDDATADIR,
                target_size=IMG_SIZE,
                color_mode="rgb",
                batch_size=BS,
                class_mode="categorical",
                shuffle=True,
                seed=42
            )

            test_generator = test_datagen.flow_from_directory(
                directory=TESTDATADIR,
                target_size=IMG_SIZE,
                color_mode="rgb",
                batch_size=1,
                class_mode=None,
                shuffle=False,
                seed=42
            )

            STEP_SIZE_TRAIN = train_generator.n // train_generator.batch_size
            STEP_SIZE_VALID = valid_generator.n // valid_generator.batch_size
            STEP_SIZE_TEST = test_generator.n // test_generator.batch_size
            model.fit_generator(generator=train_generator,
                                steps_per_epoch=STEP_SIZE_TRAIN,
                                validation_data=valid_generator,
                                validation_steps=STEP_SIZE_VALID,
                                epochs=e,
                                callbacks=[save_best, tensorboard])
            model.save(os.path.join(MODELDIR, NAME + '.h5'))

            # model.evaluate_generator(generator=valid_generator,
            #                          steps=STEP_SIZE_VALID,
            #                          verbose=1)

            print(f"\nTesting model {NAME}")

            test_generator.reset()
            pred = model.predict_generator(test_generator,
                                           steps=STEP_SIZE_TEST,
                                           verbose=1)

            export_to_excel(NAME, test_generator, pred)

            print(f"\nTesting BEST model best-{NAME}")
            model = tf.keras.models.load_model(os.path.join(MODELDIR, f"{NAME}-best.h5"))
            test_generator.reset()
            pred = model.predict_generator(test_generator,
                                           steps=STEP_SIZE_TEST,
                                           verbose=1)

            export_to_excel(NAME, test_generator, pred, "best")

            counter += 1

            # del model
            # K.clear_session()
            # cuda.select_device(0)
            # cuda.close()
            # gc.collect()
            # predicted_class_indices = np.argmax(pred, axis=1)
            # labels = train_generator.class_indices
            # labels = dict((v, k) for k, v in labels.items())
            # predictions = [labels[k] for k in predicted_class_indices]
            #
            # filenames = test_generator.filenames
            # results = pd.DataFrame({"Filename": filenames,
            #                         "Predictions": predictions})
            # results.to_csv(os.path.join(RESULTDIR, f"results-{NAME}.csv"), index=False)





    # for conv in conv_list:
    #     for dense in dense_list:
    #         # Initialize First Layer
    #         NAME = "conv-32"
    #         model = Sequential()
    #         model.add(Conv2D(32, (1, 1), input_shape=(IMG_SIZE[0], IMG_SIZE[1], 3)))
    #         model.add(Activation("relu"))
    #         NAME += "-drop25"
    #         model.add(Dropout(0.25))
    #
    #         for c_layer in conv:
    #             NAME += f"-{c_layer}"
    #             model.add(Conv2D(c_layer, (1, 1)))
    #             model.add(Activation("relu"))
    #             NAME += "-drop25"
    #             model.add(Dropout(0.25))
    #             model.add(MaxPooling2D(pool_size=(2, 2)))
    #
    #
    #
    # for conv_layer in conv_layers:
    #     for dense_layer in dense_layers:
    #
    #         # Initialize First Layer
    #         NAME = "conv-32"
    #         model = Sequential()
    #         model.add(Conv2D(32, (1, 1), input_shape=(IMG_SIZE[0], IMG_SIZE[1], 3)))
    #         model.add(Activation("relu"))
    #         NAME += "-drop25"
    #         model.add(Dropout(0.25))
    #
    #         # Multiple Conv Layers
    #         for l in range(conv_layer):
    #             NAME += "-{}".format(conv_filter_sizes[l])
    #             model.add(Conv2D(conv_filter_sizes[l], (1, 1)))
    #             model.add(Activation("relu"))
    #             NAME += "-drop25"
    #             model.add(Dropout(0.25))
    #             model.add(MaxPooling2D(pool_size=(2, 2)))
    #
    #         model.add(Flatten())
    #
    #         # Multiple Dense Layers
    #         NAME += "-dense"
    #         for l in range(dense_layer):
    #             NAME += "-{}".format(dense_filter_sizes[l])
    #             model.add(Dense(dense_filter_sizes[l]))
    #             model.add(Activation("relu"))
    #             # NAME += "-batch"
    #             # model.add(BatchNormalization())
    #             NAME += "-drop50"
    #             model.add(Dropout(0.5))
    #
    #         NAME += "-{}".format(int(time.time()))
    #         print(NAME)
    #
    #         # Final Layer
    #         model.add(Dense(4))
    #         model.add(Activation("softmax"))
    #
    #         tensorboard = TensorBoard(log_dir=os.path.join(LOGS, NAME))
    #
    #         model.compile(loss="categorical_crossentropy",
    #                       optimizer="adam",
    #                       metrics=['accuracy'])
    #
    #         save_best = ModelCheckpoint(model_name, monitor='val_loss', mode='min',
    #                                     save_best_only=True, verbose=1)
    #
    #         BS = 32
    #
    #         train_datagen = ImageDataGenerator(
    #             rescale=1. / 255,
    #             zoom_range=0.3,
    #             width_shift_range=0.1,
    #             height_shift_range=0.1,
    #             shear_range=0.15,
    #             vertical_flip=True,
    #             fill_mode="nearest")
    #
    #         valid_datagen = ImageDataGenerator(rescale=1. / 255)
    #         test_datagen = ImageDataGenerator(rescale=1. / 255)
    #
    #         train_generator = train_datagen.flow_from_directory(
    #             directory=TRAINDATADIR,
    #             target_size=IMG_SIZE,
    #             color_mode="rgb",
    #             batch_size=BS,
    #             class_mode="categorical",
    #             shuffle=True,
    #             seed=42
    #         )
    #
    #         valid_generator = valid_datagen.flow_from_directory(
    #             directory=VALIDDATADIR,
    #             target_size=IMG_SIZE,
    #             color_mode="rgb",
    #             batch_size=BS,
    #             class_mode="categorical",
    #             shuffle=True,
    #             seed=42
    #         )
    #
    #         test_generator = test_datagen.flow_from_directory(
    #             directory=TESTDATADIR,
    #             target_size=IMG_SIZE,
    #             color_mode="rgb",
    #             batch_size=1,
    #             class_mode=None,
    #             shuffle=False,
    #             seed=42
    #         )
    #
    #         STEP_SIZE_TRAIN = train_generator.n // train_generator.batch_size
    #         STEP_SIZE_VALID = valid_generator.n // valid_generator.batch_size
    #         STEP_SIZE_TEST = test_generator.n // test_generator.batch_size
    #         model.fit_generator(generator=train_generator,
    #                             steps_per_epoch=STEP_SIZE_TRAIN,
    #                             validation_data=valid_generator,
    #                             validation_steps=STEP_SIZE_VALID,
    #                             epochs=e,
    #                             callbacks=[save_best, tensorboard])
    #
    #         model.evaluate_generator(generator=valid_generator,
    #                                  steps=STEP_SIZE_VALID,
    #                                  verbose=1)
    #
    #         print("\n\nTesting model.")
    #
    #         test_generator.reset()
    #         pred = model.predict_generator(test_generator,
    #                                        steps=STEP_SIZE_TEST,
    #                                        verbose=1)
    #
    #         predicted_class_indices = np.argmax(pred, axis=1)
    #         labels = train_generator.class_indices
    #         labels = dict((v, k) for k, v in labels.items())
    #         predictions = [labels[k] for k in predicted_class_indices]
    #
    #         filenames = test_generator.filenames
    #         results = pd.DataFrame({"Filename": filenames,
    #                                 "Predictions": predictions})
    #         results.to_csv(os.path.join(RESULTDIR, f"results-{NAME}.csv"), index=False)

            # model.fit(X, y,
            #           epochs=e,
            #           validation_split=val,
            #           callbacks=[tensorboard])

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


def main():

    # create_training_data()
    # x_train, y_train, x_test, y_test = loadTrainingSet()

    conv = combinations(conv_layers, conv_filter_sizes)
    dense = combinations(dense_layers, dense_filter_sizes)

    print(conv)
    print(dense)
    trainModel(conv, dense, 10)


if __name__ == "__main__":
    main()
