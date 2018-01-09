import cv2
import numpy as np
import matplotlib.pyplot as plt


def preprocess_image(image):
    '''
    Method for preprocessing images
    '''
    # print("preprocessing image ...")
    # original shape: 160x320x3
    # apply subtle blur
    image = cv2.GaussianBlur(image, (3, 3), 0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def augment_data(image, angle):
    # data augmentation by flipping
    # print("augmenting data ...")
    image_flipped = np.fliplr(image)
    angle_flipped = -angle
    return image_flipped, angle_flipped


# visualization
def display_results(history_object):
    # plot the training and validation loss for each epoch
    plt.plot(history_object.history['loss'])
    plt.plot(history_object.history['val_loss'])
    plt.title('model mean squared error loss')
    plt.ylabel('mean squared error loss')
    plt.xlabel('epoch')
    plt.legend(['training set', 'validation set'], loc='upper right')
    plt.show()
