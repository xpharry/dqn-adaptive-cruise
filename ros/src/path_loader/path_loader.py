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

        self.frame_id = rospy.get_param('~frame_id')

        self.path_pub = rospy.Publisher('/base_path', Path, queue_size=1, latch=True)
        self.path_points_pub = rospy.Publisher('/base_path_points', PointCloud, queue_size=1, latch=True)

        self.lane_line0_pub = rospy.Publisher('/lane_line0', Path, queue_size=1, latch=True)
        self.lane_line1_pub = rospy.Publisher('/lane_line1', Path, queue_size=1, latch=True)
        self.lane_line2_pub = rospy.Publisher('/lane_line2', Path, queue_size=1, latch=True)
        self.lane_line3_pub = rospy.Publisher('/lane_line3', Path, queue_size=1, latch=True)

        self.lane_line0_points_pub = rospy.Publisher('/lane_line0_points', PointCloud, queue_size=1, latch=True)
        self.lane_line1_points_pub = rospy.Publisher('/lane_line1_points', PointCloud, queue_size=1, latch=True)
        self.lane_line2_points_pub = rospy.Publisher('/lane_line2_points', PointCloud, queue_size=1, latch=True)
        self.lane_line3_points_pub = rospy.Publisher('/lane_line3_points', PointCloud, queue_size=1, latch=True)

        self.new_path_loader(rospy.get_param('~fpath'))

        rospy.spin()

    def new_path_loader(self, path):
        if os.path.isfile(path):
            poses, points = self.load_path(path)
            lane_lines, lane_lines_points = self.draw_lane_lines(poses)
            self.publish(poses, lane_lines, points, lane_lines_points)
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
                
                pose_stamped.header.frame_id = self.frame_id
                pose_stamped.header.seq = index
                pose_stamped.pose = pose
                poses.append(pose_stamped)

                point = Point32()
                point.x = float(wp['x'])
                point.y = float(wp['y'])
                point.z = 0.0
                points.append(point)

        return poses, points

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    def closest_waypoint(self, x, y, poses):
        closest_len = 100000  # large number
        closest_waypoint = 0

        for i in range(len(poses)):
            map_x = poses[i].pose.position.x
            map_y = poses[i].pose.position.y
            dist = self.distance(x, y, map_x, map_y)
            if dist < closest_len:
                closest_len = dist
                closest_waypoint = i

        return closest_waypoint

    def next_waypoint(self, x, y, psi, poses):
        closest_waypoint = self.closest_waypoint(x, y, poses)

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
    def get_frenet(self, x, y, psi, poses):
        next_wp = self.next_waypoint(x, y, psi, poses)

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

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        # see if d value is positive or negative by comparing it to a center point

        center_x = 1000 - poses[prev_wp].pose.position.x
        center_y = 2000 - poses[prev_wp].pose.position.y
        center_to_pos = self.distance(center_x, center_y, x_x, x_y)
        center_to_ref = self.distance(center_x, center_y, proj_x, proj_y)

        if center_to_pos <= center_to_ref:
            frenet_d *= -1

        # calculate s value
        frenet_s = 0
        for i in range(prev_wp):
            frenet_s += self.distance(poses[i].pose.position.x, poses[i].pose.position.y, poses[i+1].pose.position.x, poses[i+1].pose.position.y)

        frenet_s += self.distance(0, 0, proj_x, proj_y)

        return frenet_s, frenet_d

    # Transform from Frenet s,d coordinates to Cartesian x,y
    def get_xy(self, s, d, maps_s, poses):
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

    def save_lane_line(self, fname, maps_s, d, poses):
        with open(fname, 'w') as wfile:
            writer = csv.writer(wfile, delimiter=' ')
            for i in range(len(maps_s)):
                if i == 0:
                    x, y = self.get_xy(maps_s[i]+0.00001, d, maps_s, poses)
                else:
                    x, y = self.get_xy(maps_s[i], d, maps_s, poses)

                writer.writerow([x, y, 0, 0])

    def draw_lane_lines(self, poses):
        maps_s = []
        maps_s.append(0)
        map_x_prev = poses[0].pose.position.x
        map_y_prev = poses[0].pose.position.y
        for i in range(1, len(poses)):
            map_x = poses[i].pose.position.x
            map_y = poses[i].pose.position.y
            map_yaw = math.atan2(map_y - map_y_prev, map_x - map_x_prev)
            map_s, map_d = self.get_frenet(map_x, map_y, map_yaw, poses)
            maps_s.append(map_s)
            map_x_prev = map_x
            map_y_prev = map_y

        # debug
        # for i in range(len(maps_s)):
        #     rospy.loginfo('i = %d, map_s = %f' % (i, maps_s[i]))

        nlines = 4

        lane_lines = []
        lane_lines_points = []

        for iline in range(nlines):
            lane_line = []
            lane_line_points = []
            d = iline * 4
            for i in range(len(maps_s)):
                if i == 0:
                    x, y = self.get_xy(maps_s[i]+0.00001, d, maps_s, poses)
                else:
                    x, y = self.get_xy(maps_s[i], d, maps_s, poses)
                # debug
                # rospy.loginfo('map_x = %f, map_y = %f, x = %f, y = %f' % (poses[i].pose.position.x, poses[i].pose.position.y, x, y))

                pose_stamped = PoseStamped()
                pose = Pose()

                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                
                pose_stamped.header.frame_id = self.frame_id
                pose_stamped.header.seq = i
                pose_stamped.pose = pose

                lane_line.append(pose_stamped)

                point = Point32()
                point.x = x
                point.y = y
                point.z = 0.0
                lane_line_points.append(point)

            lane_lines.append(lane_line)
            lane_lines_points.append(lane_line_points)

        # self.save_lane_line(rospy.get_param('~new_fpath'), maps_s, -6, poses)

        return lane_lines, lane_lines_points

    def publish(self, poses, lane_lines, points, lane_lines_points):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time(0)
        path.poses = poses
        self.path_pub.publish(path)

        # lane lines

        lane_line0 = Path()
        lane_line0.header.frame_id = self.frame_id
        lane_line0.header.stamp = rospy.Time(0)
        lane_line0.poses = lane_lines[0]
        self.lane_line0_pub.publish(lane_line0)

        lane_line1 = Path()
        lane_line1.header.frame_id = self.frame_id
        lane_line1.header.stamp = rospy.Time(0)
        lane_line1.poses = lane_lines[1]
        self.lane_line1_pub.publish(lane_line1)

        lane_line2 = Path()
        lane_line2.header.frame_id = self.frame_id
        lane_line2.header.stamp = rospy.Time(0)
        lane_line2.poses = lane_lines[2]
        self.lane_line2_pub.publish(lane_line2)

        lane_line3 = Path()
        lane_line3.header.frame_id = self.frame_id
        lane_line3.header.stamp = rospy.Time(0)
        lane_line3.poses = lane_lines[3]
        self.lane_line3_pub.publish(lane_line3)

        path_points = PointCloud()
        path_points.header.frame_id = self.frame_id
        path_points.header.stamp = rospy.Time(0)
        path_points.points = points
        self.path_points_pub.publish(path_points)

        # lane lines point cloud display

        lane_line0_points = PointCloud()
        lane_line0_points.header.frame_id = self.frame_id
        lane_line0_points.header.stamp = rospy.Time(0)
        lane_line0_points.points = lane_lines_points[0]
        self.lane_line0_points_pub.publish(lane_line0_points)

        lane_line1_points = PointCloud()
        lane_line1_points.header.frame_id = self.frame_id
        lane_line1_points.header.stamp = rospy.Time(0)
        lane_line1_points.points = lane_lines_points[1]
        self.lane_line1_points_pub.publish(lane_line1_points)

        lane_line2_points = PointCloud()
        lane_line2_points.header.frame_id = self.frame_id
        lane_line2_points.header.stamp = rospy.Time(0)
        lane_line2_points.points = lane_lines_points[2]
        self.lane_line2_points_pub.publish(lane_line2_points)

        lane_line3_points = PointCloud()
        lane_line3_points.header.frame_id = self.frame_id
        lane_line3_points.header.stamp = rospy.Time(0)
        lane_line3_points.points = lane_lines_points[3]
        self.lane_line3_points_pub.publish(lane_line3_points)

if __name__ == '__main__':
    try:
        PathLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start path loader node.')
