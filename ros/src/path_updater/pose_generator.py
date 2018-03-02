#!/usr/bin/env python

"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   Feb 21 2018
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point32
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
import path_utils as utils

'''
This node will publish the car's current position.
'''

LOOKAHEAD_WPS = 200  # Number of paths we will publish. You can change this number


class poseGenerator(object):
    def __init__(self):
        rospy.init_node('pose_generator')

        rospy.Subscriber('/base_path', Path, self.base_path_cb)

        self.pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1)

        self.pose = None
        self.frame_id = None
        self.base_path = None

        self.run()

    def base_path_cb(self, msg):
        """ Store the given map """
        self.base_path = msg

    def run(self):
        """
        Continuously publish a pose as a fake vehicle
        """
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            if self.base_path is None:
                continue

            seq = 0
            if self.pose is None:
                next_pose = self.base_path.poses[seq]
            else:
                seq = utils.get_closest_waypoint_index(self.pose, self.base_path.poses)
                next_pose = self.base_path.poses[(seq+1)%(len(self.base_path.poses))]
            self.pose = next_pose

            # rospy.loginfo('Pose Seq = %d, x: %f, y % f' % (seq, self.pose.pose.position.x, self.pose.pose.position.y))

            # Publish
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'world'
            pose_stamped.header.seq = seq + 1
            pose_stamped.pose = next_pose.pose
            self.pose_pub.publish(pose_stamped)

            rate.sleep()


if __name__ == '__main__':
    try:
        poseGenerator()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')
