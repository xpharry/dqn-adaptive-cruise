# general
from __future__ import absolute_import
import os
import sys
import argparse
import csv
import cv2
import numpy as np
import tensorflow as tf
import sklearn
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers import Flatten, Dense, Conv2D, MaxPooling2D, Dropout, Lambda, Cropping2D
from utils import preprocess_image, augment_data, display_results

FLAGS = None
data_dir = './data'


# read data
def read_data(data_path):
    samples = []
    log_path = os.path.join(os.path.abspath(data_path), 'scan_data.csv')
    with open(log_path) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)
    return samples


def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1:  # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            scans = []
            brakes = []
            for batch_sample in batch_samples:
                scan = batch_sample[:-1]
                brake = batch_sample[-1]
                scans.append(scan)
                brakes.append(brake)

            # trim image to only see section with road
            X_train = np.array(scans)
            y_train = np.array(brakes)

            yield sklearn.utils.shuffle(X_train, y_train)


# create model
def create_model():
    input_shape = (181,)
    model = Sequential()
    # model.add(Cropping1D(cropping=((50, 20), (0, 0))))
    # model.add(Conv1D(6, (5, 5), strides=(2, 2), activation='relu'))
    # model.add(MaxPooling1D(strides=(2, 2)))
    # model.add(Conv1D(16, (5, 5), activation='relu'))
    # model.add(MaxPooling2D(strides=(2, 2)))
    # model.add(Flatten())
    model.add(Dense(120, activation='relu', input_shape=input_shape))
    model.add(Dropout(0.5))
    model.add(Dense(84, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(1))
    return model


def train(model, train_samples, validation_samples):
    # model.summary()
    model.compile(loss='mse', optimizer='adam')
    # model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=5, verbose=1)

    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=FLAGS.batch_size)
    validation_generator = generator(validation_samples, batch_size=FLAGS.batch_size)

    # train
    history_object = model.fit_generator(train_generator,
                                         steps_per_epoch=len(train_samples) // FLAGS.batch_size,
                                         validation_data=validation_generator,
                                         validation_steps=len(validation_samples) // FLAGS.batch_size,
                                         epochs=FLAGS.epochs,
                                         verbose=1)
    model.save('model.h5')

    # print the keys contained in the history object
    print(history_object.history.keys())

    return history_object


def main(_):
    # read data
    samples = read_data(FLAGS.data_dir)
    print("samples size = ", len(samples))
    # print("a typical sample is like:")
    # print(samples[0])

    # data split
    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    # print(train_samples[0])

    # create model
    model = create_model()

    # train
    history_object = train(model, train_samples, validation_samples)

    # display
    display_results(history_object)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Directory Parameters:
    parser.add_argument('--data_dir', type=str, default=data_dir,
                        help='Input Data Directory')
    parser.add_argument('--epochs', type=int, default=5,
                        help='The number of epochs')
    parser.add_argument('--batch_size', type=int, default=32,
                        help='The batch size')

    FLAGS, unparsed = parser.parse_known_args()
    tf.app.run(main=main, argv=[sys.argv[0]] + unparsed)

"""
Example:
python train_with_generator.py \
--data_dir ./data/ \
--epochs 5 \
--batch_size 128
"""