import os
import csv
from math import sin, cos, pi
import tf
import rospy
from geometry_msgs.msg import Quaternion, Point32, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path


RADIUS = 36.6
NEDGES = 301


# utility fnc to compute min delta angle, accounting for periodicity
def min_dang(dang):
    while dang > pi: dang -= 2.0 * pi
    while dang < -pi: dang += 2.0 * pi
    return dang


def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0., 0., yaw)


def generate_standard_track(radius, nedges):
    poses = []

    # straight line
    for i in range(nedges):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = i*radius/nedges - radius
        pose.position.y = 0.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = i + nedges
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # circle
    phi0 = -pi/2
    for i in range(nedges):
        pose_stamped = PoseStamped()
        pose = Pose()

        phi = min_dang(phi0 + 2.0*pi/nedges * i)
        pose.position.x = radius * cos(phi)
        pose.position.y = radius * sin(phi) + radius
        pose.position.z = 0.0  # let's hope so!

        yaw = min_dang(phi + pi/2.0)
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.pose = pose
        poses.append(pose_stamped)


    return poses


def write_to_csv(poses):
    fname = 'circle_track_map.csv'
    with open(fname, 'w') as wfile:
        writer = csv.writer(wfile, delimiter=' ')
        for pose in poses:
            writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0])


def main():
    poses = generate_standard_track(RADIUS, NEDGES)
    write_to_csv(poses)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')