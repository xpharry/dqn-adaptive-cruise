#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import csv


class trajRecorder(object):

    def __init__(self):
        rospy.init_node('traj_recorder')

        self.pose = None
        self.timestamp = None

        rospy.Subscriber('current_pose', PoseStamped, self.pose_cb)

        self.run()

    def pose_cb(self, msg):
        self.pose = msg.pose
        # rospy.loginfo("pose is received ...")

    def run(self):

        fname = rospy.get_param('~filepath', 'test_track_waypoints.csv')
        with open(fname, 'w') as wfile:
            writer = csv.writer(wfile, delimiter=' ')
            writer.writerow([0, 0, 0, 0])

            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                if self.pose is None:
                    rospy.loginfo("pose is none ...")
                else:
                    writer.writerow([self.pose.position.x, self.pose.position.y, self.pose.position.z, 0])
                    print(self.pose.position.x, self.pose.position.y, self.pose.position.z, 0)
                rate.sleep()


if __name__ == '__main__':
    try:
        trajRecorder()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')