"""
Helper functions for paths_updater

Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 20 2018
"""

import math
from math import cos, sin, sqrt
from copy import deepcopy

import rospy
import numpy as np
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point32
from sensor_msgs.msg import PointCloud

from scipy.interpolate import interp1d


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


def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))


def closest_waypoint_index(x, y, poses):
    closest_len = 100000  # large number
    closest_waypoint = 0

    for i in range(len(poses)):
        map_x = poses[i].pose.position.x
        map_y = poses[i].pose.position.y
        dist = distance(x, y, map_x, map_y)
        if dist < closest_len:
            closest_len = dist
            closest_waypoint = i

    return closest_waypoint


def next_waypoint(x, y, psi, poses):
    closest_waypoint = closest_waypoint_index(x, y, poses)

    map_x = poses[closest_waypoint].pose.position.x
    map_y = poses[closest_waypoint].pose.position.y

    heading = math.atan2((map_y-y), (map_x-x))

    angle = math.fabs(psi-heading)
    angle = min(2*math.pi - angle, angle);

    if angle > math.pi/4:
      closest_waypoint += 1
      if closest_waypoint == len(poses):
        closest_waypoint = 0

    return closest_waypoint


# Transform from Cartesian x,y coordinates to Frenet s,d coordinates
# vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
def get_frenet(x, y, psi, poses):
    next_wp = next_waypoint(x, y, psi, poses)

    prev_wp = next_wp-1
    if next_wp == 0:
        prev_wp  = len(poses)-1

    n_x = poses[next_wp].pose.position.x - poses[prev_wp].pose.position.x
    n_y = poses[next_wp].pose.position.y - poses[prev_wp].pose.position.y
    x_x = x - poses[prev_wp].pose.position.x
    x_y = y - poses[prev_wp].pose.position.y

    # find the projection of x onto n
    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
    proj_x = proj_norm*n_x
    proj_y = proj_norm*n_y

    frenet_d = distance(x_x, x_y, proj_x, proj_y)

    # see if d value is positive or negative by comparing it to a center point

    center_x = 1000 - poses[prev_wp].pose.position.x
    center_y = 2000 - poses[prev_wp].pose.position.y
    center_to_pos = distance(center_x, center_y, x_x, x_y)
    center_to_ref = distance(center_x, center_y, proj_x, proj_y)

    if center_to_pos <= center_to_ref:
        frenet_d *= -1

    # calculate s value
    frenet_s = 0
    for i in range(prev_wp):
        frenet_s += distance(poses[i].pose.position.x, poses[i].pose.position.y, poses[i+1].pose.position.x, poses[i+1].pose.position.y)

    frenet_s += distance(0, 0, proj_x, proj_y)

    return frenet_s, frenet_d


# Transform from Frenet s,d coordinates to Cartesian x,y
def get_xy(s, d, maps_s, poses):
    prev_wp = -1

    while s > maps_s[prev_wp+1] and prev_wp < len(maps_s)-1:
        prev_wp += 1

    wp2 = (prev_wp+1) % len(maps_s)

    heading = math.atan2((poses[wp2].pose.position.y-poses[prev_wp].pose.position.y), (poses[wp2].pose.position.x-poses[prev_wp].pose.position.x))
    # the x,y,s along the segment
    seg_s = s-maps_s[prev_wp]

    seg_x = poses[prev_wp].pose.position.x + seg_s*math.cos(heading)
    seg_y = poses[prev_wp].pose.position.y + seg_s*math.sin(heading)

    perp_heading = heading-math.pi/2

    x = seg_x + d*math.cos(perp_heading)
    y = seg_y + d*math.sin(perp_heading)

    return x, y


def get_next_waypoints(waypoints, current_pose, base, n, ilane):
    """Return a list of n paths ahead of the vehicle"""
    frame_id = waypoints[0].header.frame_id

    next_waypoints = []
    for k in range(base, base+4*n):
        wp = k % len(waypoints)
        next_waypoints.append(waypoints[wp])

    # frenet transform
    maps_s = []
    map_s = 0
    maps_s.append(map_s)
    map_x_prev = next_waypoints[0].pose.position.x
    map_y_prev = next_waypoints[0].pose.position.y
    for i in range(1, 4*n):
        map_x = next_waypoints[i].pose.position.x
        map_y = next_waypoints[i].pose.position.y
        map_s += distance(map_x, map_y, map_x_prev, map_y_prev)
        maps_s.append(map_s)
        map_x_prev = map_x
        map_y_prev = map_y

    # debug
    # for i in range(len(maps_s)):
    #     rospy.loginfo('i = %d, map_s = %f' % (i, maps_s[i]))

    current_x, current_y = current_pose.pose.position.x, current_pose.pose.position.y
    # current_s, current_d = get_frenet(current_x, current_y, 0, next_waypoints)
    # rospy.loginfo('******** s = %f, d = %f *********' % (current_s, current_d))
    current_d = distance(current_x, current_y, next_waypoints[0].pose.position.x, next_waypoints[0].pose.position.y)

    d = 2 + ilane * 4

    # fits a polynomial for given paths
    s_coords = [maps_s[0], maps_s[1], maps_s[n/2], maps_s[-3], maps_s[-2], maps_s[-1]]
    d_coords = [d, d, d, d, d, d]

    f = interp1d(s_coords, d_coords, kind='cubic')

    # construct lane change path
    x_points = []
    y_points = []

    target_s = min(30.0, maps_s[-2])
    target_d = d

    s_add_on = 0
    
    for i in range(n):
        s_point = s_add_on + target_s / n
        d_point = f(s_point)

        # rospy.loginfo('s = %f, d = %f' % (s_point, d_point))

        s_add_on = s_point

        x_point, y_point = get_xy(s_point, d_point, maps_s, next_waypoints)

        x_points.append(x_point)
        y_points.append(y_point)

    next_waypoints = []
    next_waypoints_cloud = PointCloud()
    next_waypoints_cloud.header.frame_id = frame_id

    for i in range(n):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = x_points[i]
        pose.position.y = y_points[i]
        pose.position.z = 0.5
        
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.seq = i
        pose_stamped.pose = pose

        next_waypoints.append(pose_stamped)

        point = Point32()
        point.x = x_points[i]
        point.y = y_points[i]
        point.z = 0.5
        next_waypoints_cloud.points.append(point)

        # rospy.loginfo('x = %f, y = %f' % (x_points[i], y_points[i]))

    return next_waypoints, next_waypoints_cloud

