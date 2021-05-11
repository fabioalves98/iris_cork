import numpy as np
import cv2
import os
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session


# DO NOT MODIFY - Classification classes - DO NOT MODIFY
CATEGORIES = ["Back", "Belly", "SideBellyLeft", "SideBellyRight"]


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
    img = np.expand_dims(img, axis=3)

    return img


def loadCNN(file):
    """load .h5 model file

    :param file: path to the file
    :return: keras pre-trained model
    """
    return tf.keras.models.load_model(file)


def classify(img, model, gpu=False):
    """ Classify the image using the main model and the side model. Cork angles must be handled after.

    :param gpu: Use GPU for classification
    :param img: Image to classify
    :param model: Model for the classification (Back, Belly, SideBellyRight, SideBellyLeft)
    :return: Classification of the model (category, accuracy)
    """
    # Prepare GPU if choosed
    if gpu:
        # initialize gpu components
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
        config.log_device_placement = True  # to log device placement (on which device the operation ran)
        sess = tf.Session(config=config)
        set_session(sess)  # set this TensorFlow session as the default session for Keras
    else:
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
        os.environ["CUDA_VISIBLE_DEVICES"] = ""

    # Prepare the image for CNN classification
    img = prepareIMG(img)
    # Load model from file
    try:
        model = loadCNN(model)
    except Exception as e:
        print(e)
        print("Cannot load model file. Using default path.")

    prediction = model.predict(img)
    prediction_accuracy = round(float(np.amax(prediction)) * 100, 2)
    prediction_category = CATEGORIES[np.argmax(prediction)]

    classification = (prediction_category, prediction_accuracy)
    return classification
