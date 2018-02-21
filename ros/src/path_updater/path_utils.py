"""
Helper functions for paths_updater

Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 20 2018
"""
# pylint: disable=invalid-name

from math import cos, sin, sqrt
from copy import deepcopy

import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point32
from sensor_msgs.msg import PointCloud


def construct_path_object(frame_id, waypoints):
    """ Path object contains the list of final paths ahead with velocity"""
    path = Path()
    path.header.frame_id = frame_id
    path.poses = waypoints
    path.header.stamp = rospy.Time.now()
    return path


def get_euler(pose):
    """ Returns the roll, pitch yaw angles from a Quaternion \
    Args:
        pose: geometry_msgs/Pose.msg

    Returns:
        roll (float), pitch (float), yaw (float)
    """
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


def get_squared_distance(p1, p2):
    """Returns squared euclidean distance between two 2D points"""
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return dx * dx + dy * dy


def is_waypoint_behind(pose, waypoint):
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose

    Args:
        pose (object) : A pose object
        paths (object) : A path object

    Returns:
        bool : True if the path is behind the car else False

    """
    _, _, yaw = get_euler(pose.pose)
    originX = pose.pose.position.x
    originY = pose.pose.position.y

    shift_x = waypoint.pose.position.x - originX
    shift_y = waypoint.pose.position.y - originY

    x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

    if x > 0:
        return False
    return True


def get_closest_waypoint_index(pose, poses):
    """
    pose: geometry_msg.msgs.Pose instance
    paths: list of styx_msgs.msg.path instances
    returns index of the closest path in the list paths
    """
    best_gap = float('inf')
    best_index = 0
    my_position = pose.pose.position

    for i, pose in enumerate(poses):

        other_position = pose.pose.position
        gap = get_squared_distance(my_position, other_position)

        if gap < best_gap:
            best_index, best_gap = i, gap

    is_behind = is_waypoint_behind(pose, poses[best_index])
    if is_behind:
        best_index += 1
    return best_index


def get_next_waypoints(waypoints, i, n):
    """Return a list of n paths ahead of the vehicle"""
    next_waypoints = []
    next_waypoints_cloud = PointCloud()
    next_waypoints_cloud.header.frame_id = '/world'

    for k in range(i, i+n):
        wp = k % len(waypoints)

        next_waypoints.append(waypoints[wp])

        point = Point32()
        point.x = waypoints[wp].pose.position.x
        point.y = waypoints[wp].pose.position.y
        point.z = waypoints[wp].pose.position.z

        next_waypoints_cloud.points.append(point)

    return next_waypoints, next_waypoints_cloud


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given paths"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """calculates the radius of curvature"""
    if coeffs is None:
        return None
    a = coeffs[0]
    b = coeffs[1]
    return (1 + (2 * a * X + b) ** 2) ** 1.5 / np.absolute(2 * a)
