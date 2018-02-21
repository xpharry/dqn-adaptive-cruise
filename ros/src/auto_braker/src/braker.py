import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np
from keras.models import load_model
import h5py
from keras import __version__ as keras_version

model = None
ranges = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
speed = 0

def scan_callback(data):
    ranges = list(data.ranges)


def vel_callback(data):
    speed = data.linear.x


def compute_speed(brake):
    if brake:
        des_speed = speed - 0.25
        if des_speed < 0:
            des_speed = 0
    else:
        des_speed = speed + 0.25
        if des_speed < 5.0:
            des_speed = 5.0
    return des_speed

def braker():
    # ros
    # unique node name
    rospy.init_node('braker', anonymous=True)
    # subscriber and publisher
    rospy.Subscriber("/catvehicle2/front_laser_points", LaserScan, scan_callback)
    rospy.Subscriber("/catvehicle2/cmd_vel", Twist, vel_callback)
    pub = rospy.Publisher('/catvehicle2/des_speed', Float64, queue_size=100)

    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        scan_data = ranges[:]
        scan_data.append(speed)
        scan_data = np.array(scan_data)
        X = scan_data.transpose()
        brake = float(model.predict(X[None,:], batch_size=1))
        print "##### Brake or not: %d #####", brake
        cmd_speed = compute_speed(brake)
        pub.publish(cmd_speed)
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Auto Brake')
    parser.add_argument(
        'model',
        type=str,
        help='Path to model h5 file. Model should be on the same path.'
    )
    args = parser.parse_args()

    # check that model Keras version is same as local Keras version
    f = h5py.File(args.model, mode='r')
    model_version = f.attrs.get('keras_version')
    keras_version = str(keras_version).encode('utf8')

    if model_version != keras_version:
        print('You are using Keras version ', keras_version,
              ', but the model was built using ', model_version)

    model = load_model(args.model)

    braker()

'''
Example:
python code/braker.py model.h5 
'''





