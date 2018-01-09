# general
import os
import sys
import argparse
import csv
import numpy as np
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Flatten, Dense, Conv2D, MaxPool2D, Dropout, Lambda, Cropping2D
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

    scans = []
    brakes = []
    for line in samples:
        scan = line[:-1]
        brake = line[-1]
        scans.append(scan)
        brakes.append(brake)
    return np.array(scans), np.array(brakes)


# create model
def create_model():
    input_shape = (181,)
    model = Sequential()
    model.add(Dense(120, activation='relu', input_shape=input_shape))
    model.add(Dropout(0.5))
    model.add(Dense(84, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(1, activation="sigmoid"))
    return model


def train(model, X_train, y_train):
    model.summary()
    model.compile(loss='mse', optimizer='adam')
    history_object = model.fit(X_train, y_train, validation_split=0.2, shuffle=True, epochs=FLAGS.epochs, verbose=1)
    model.save('model.h5')

    # print the keys contained in the history object
    print(history_object.history.keys())

    return history_object


def main(_):
    # read data
    X_train, y_train = read_data(FLAGS.data_dir)

    print(X_train.shape, y_train.shape)

    # create model
    model = create_model()

    # train
    history_object = train(model, X_train, y_train)

    # display
    # display_results(history_object)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Directory Parameters:
    parser.add_argument('--data_dir', type=str, default=data_dir,
                        help='Input Data Directory')
    parser.add_argument('--epochs', type=int, default=5,
                        help='The number of epochs')

    FLAGS, unparsed = parser.parse_known_args()
    tf.app.run(main=main, argv=[sys.argv[0]] + unparsed)

"""
Example:
python train_with_fit.py \
--data_dir ./data/ \
--epochs 5
"""