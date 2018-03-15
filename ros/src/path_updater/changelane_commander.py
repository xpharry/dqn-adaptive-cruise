#!/usr/bin/env python

"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   Mar 14 2018
"""

import random
import rospy
from std_msgs.msg import String

'''
This node will publish commands to change lane.

rostopic pub /ego/change_lane std_msgs/String Left -1

'''

class changelaneCommander(object):
    def __init__(self):

        rospy.init_node('changelane_commander')

        self.change_lane_pub = rospy.Publisher('ego/change_lane', String, queue_size=1)

        self.run()

    def run(self):
        """
        Continuously publish a pose as a fake vehicle
        """
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown():

            command = random.choice(["Keep", "Left", "Right"])

            self.change_lane_pub.publish(command)

            rospy.loginfo("The command of lane change is %s" % command)

            rate.sleep()


if __name__ == '__main__':
    try:
        changelaneCommander()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')
