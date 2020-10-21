import cv2
import os
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session

from Classifier import classify


RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "res")
print(RESOURCES)

def main():
    # Choose a model from "models" folder in resources
    model = os.path.join(RESOURCES, "models", "conv-32-drop25-dense-128-drop50-64-drop50-1572285048.h5")
    print(model)

    # Demo images
    images_path = []
    try:
        images_path = [os.path.join(os.path.join(RESOURCES, "demo_images"), f) for f in os.listdir(os.path.join(RESOURCES, "demo_images"))]
    except Exception:
        pass

    for img in images_path:
        # Read image from file
        image = cv2.imread(img)
        cv2.imshow("Demo Classification v0.1", image)

        # image = image to classify, model = path to model,
        # gpu = enable GPU for a single classification (for better performance enable on main function)
        (classification_category, classification_accuracy) = classify(image, model)

        # Show image and classification
        print(f"Category: {classification_category}\n"
              f"Accuracy: {classification_accuracy}\n"
              f"""-------------------------""")
        print("Press Enter to continue...\n")
        cv2.waitKey(0)

    pass


if __name__ == "__main__":
    # Enable GPU - don't use GPU parameter on function "classify"
    # # initialize gpu components
    # config = tf.ConfigProto()
    # config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
    # config.log_device_placement = True  # to log device placement (on which device the operation ran)
    # sess = tf.Session(config=config)
    # set_session(sess)  # set this TensorFlow session as the default session for Keras
    main()
