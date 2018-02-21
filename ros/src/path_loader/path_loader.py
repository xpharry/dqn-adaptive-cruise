#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, Point32, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

import tf
import rospy

CSV_HEADER = ['x', 'y', 's', 'dx', 'dy']
MAX_DECEL = 1.0


class PathLoader(object):

    def __init__(self):
        rospy.init_node('path_loader', log_level=rospy.DEBUG)
        self.path_pub = rospy.Publisher('/base_path', Path, queue_size=1, latch=True)
        self.path_points_pub = rospy.Publisher('/base_path_points', PointCloud, queue_size=1, latch=True)
        self.new_path_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_path_loader(self, path):
        if os.path.isfile(path):
            poses, points = self.load_path(path)
            self.publish(poses, points)
            rospy.loginfo('Path Loaded')
        else:
            rospy.logerr('%s is not a file', path)

    def load_path(self, fname):
        poses = []
        points = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, delimiter=' ', fieldnames=CSV_HEADER)
            for index, wp in enumerate(reader):
                pose_stamped = PoseStamped()
                pose = Pose()

                pose.position.x = float(wp['x'])
                pose.position.y = float(wp['y'])
                pose.position.z = 0.0
                
                pose_stamped.header.frame_id = 'world'
                pose_stamped.header.seq = index
                pose_stamped.pose = pose
                poses.append(pose_stamped)

                point = Point32()
                point.x = float(wp['x'])
                point.y = float(wp['y'])
                point.z = 0.0
                points.append(point)

        return poses, points

    def publish(self, poses, points):
        path = Path()
        path.header.frame_id = '/world'
        path.header.stamp = rospy.Time(0)
        path.poses = poses
        self.path_pub.publish(path)

        path_points = PointCloud()
        path_points.header.frame_id = '/world'
        path_points.header.stamp = rospy.Time(0)
        path_points.points = points
        self.path_points_pub.publish(path_points)


if __name__ == '__main__':
    try:
        PathLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start path loader node.')
