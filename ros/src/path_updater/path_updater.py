#!/usr/bin/env python

"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 20 2018
"""


import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Float64, Int32, String
import path_utils as utils
import math

from scipy.interpolate import interp1d


'''
This node will publish paths from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50  # Number of paths we will publish. You can change this number


class pathUpdater(object):
    def __init__(self):
        rospy.init_node('path_updater')

        self.pose_stamped = None
        self.pose = None
        self.frame_id = None
        self.base_path = None
        self.waypoints = None
        self.cruise_speed = None
        self.ilane = None
        self.lane_timestamp = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_path', Path, self.base_path_cb)
        rospy.Subscriber('/cruise_speed', Float64, self.cruise_speed_cb)
        rospy.Subscriber('/change_lane', String, self.change_lane_cb)

        self.final_path_pub = rospy.Publisher('final_path', Path, queue_size=1)
        self.final_path_points_pub = rospy.Publisher('final_path_points', PointCloud, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)
        self.current_lane_pub = rospy.Publisher('current_lane', Int32, queue_size=1)
        self.change_lane_reward_pub = rospy.Publisher('change_lane_reward', Int32, queue_size=1)

        self.run()

    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose_stamped = msg
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id

    def base_path_cb(self, msg):
        """ Store the given map """
        self.base_path = msg

    def cruise_speed_cb(self, msg):
        self.cruise_speed = msg.data

    def change_lane_cb(self, msg):
        """ Store the given map """
        signal = msg.data
        reward = 0
        new_timestamp = rospy.get_time()
        if self.lane_timestamp == None or new_timestamp - self.lane_timestamp > 0:
            if signal == "Keep":
                self.lane_timestamp = new_timestamp
                reward = 10
                # print("\nKeep lane!")
            elif signal == "Left" and self.ilane > 0:
                self.ilane -= 1
                self.lane_timestamp = new_timestamp
                reward = 10
                # print("\nChanged to Left!")
            elif signal == "Right" and self.ilane < 2:
                self.ilane += 1
                self.lane_timestamp = new_timestamp
                reward = 10
                # print("\nChanged to Right!")
            else:
                reward = -10
                # print("Cannot execute the command this time because the target lane is not available ...")
        else:
            reward = -5
            # print("Cannot execute the command this time because the last execution hasn't finished ...")
        self.current_lane_pub.publish(self.ilane)
        self.change_lane_reward_pub.publish(reward)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp, velocity):
        waypoints[wp].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def run(self):
        """
        Continuously publish local path paths with target velocities
        """
        rate = rospy.Rate(50)

        self.ilane = int(rospy.get_param("~lane_index", '0'))
        self.cruise_speed = int(rospy.get_param("~cruise_speed", '10'))

        while not rospy.is_shutdown():

            if self.base_path is None or self.pose is None or self.frame_id is None:
                continue

            # Where in base paths list the car is
            car_index = utils.get_closest_waypoint_index(self.pose_stamped, self.base_path.poses)

            # Get subset paths ahead
            lookahead_waypoints, lookahead_waypoints_display \
                = utils.get_next_waypoints(self.base_path.poses, self.pose_stamped, car_index, LOOKAHEAD_WPS, self.ilane)

            # Publish
            path = utils.construct_path_object(self.frame_id, lookahead_waypoints)
            
            lane = Lane()
            lane.header.frame_id = self.frame_id
            for i in range(len(lookahead_waypoints)):
                waypoint = Waypoint()
                waypoint.pose = lookahead_waypoints[i]
                waypoint.twist.header.frame_id = self.frame_id
                waypoint.twist.twist.linear.x = self.cruise_speed
                lane.waypoints.append(waypoint)

            rospy.logdebug('Update local path and waypoints ...')

            self.final_path_pub.publish(path)
            self.final_path_points_pub.publish(lookahead_waypoints_display)
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

            rate.sleep()


if __name__ == '__main__':
    try:
        pathUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start path updater node.')
