import time
import sys
import rospy
import roslaunch
import numpy as np
import math

import gym
from gym.utils import seeding
from gym import utils, spaces
from gym_vehicle.envs import gazebo_env

from std_msgs.msg import Float64, Int32, String
from std_srvs.srv import Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates

MAX_SPEED = 22.35  # m/sec; tune this
COLLISON_DIST = 3 # m
INIT_LANE_INDEX = 1


class GazeboStandardTrackMultiVehicleLidarNnEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboStandardTrackMultiVehicleLidar_v0.launch")

        self.base_path = None
        self.speeds = [0, 0, 0, 0, 0, 0]
        self.poses = [None, None, None, None, None, None]
        self.path_dists = [80, 80, 80, 80, 80]
        self.lane_index = INIT_LANE_INDEX
        self.lanes = [0, 0, 0]
        self.travel_dist = 0
        self.prev_ego_pose = None
        self.change_lane_reward = 0

        rospy.Subscriber('/base_path', Path, self.base_path_cb)

        rospy.Subscriber('/fusion1/twist', TwistStamped, self.fusion1_vel_cb)
        rospy.Subscriber('/fusion1/twist', TwistStamped, self.fusion2_vel_cb)
        rospy.Subscriber('/ego/twist', TwistStamped, self.ego_vel_cb)
        rospy.Subscriber('/mondeo2/twist', TwistStamped, self.mondeo2_vel_cb)
        rospy.Subscriber('/mkz1/twist', TwistStamped, self.mkz1_vel_cb)
        rospy.Subscriber('/mkz2/twist', TwistStamped, self.mkz2_vel_cb)

        rospy.Subscriber('/fusion1/current_pose', PoseStamped, self.fusion1_pose_cb)
        rospy.Subscriber('/fusion1/current_pose', PoseStamped, self.fusion2_pose_cb)
        rospy.Subscriber('/ego/current_pose', PoseStamped, self.ego_pose_cb)
        rospy.Subscriber('/mondeo2/current_pose', PoseStamped, self.mondeo2_pose_cb)
        rospy.Subscriber('/mkz1/current_pose', PoseStamped, self.mkz1_pose_cb)
        rospy.Subscriber('/mkz2/current_pose', PoseStamped, self.mkz2_pose_cb)

        rospy.Subscriber('/ego/current_lane', Int32, self.cur_lane_cb)
        rospy.Subscriber('/ego/chang_lane_reward', Int32, self.change_lane_reward_cb)

        self.cruise_speed_pub = rospy.Publisher('/ego/cruise_speed', Float64, queue_size=5)
        self.change_lane_pub = rospy.Publisher('/ego/chang_lane', String, queue_size=5)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.action_space = spaces.Discrete(21)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def base_path_cb(self, msg):
        """ Store the given map """
        self.base_path = msg

    def fusion1_vel_cb(self, data):
        self.speeds[0] = data.twsit.linear.x

    def fusion2_vel_cb(self, data):
        self.speeds[1] = data.twist.linear.x

    def ego_vel_cb(self, data):
        self.speeds[2] = data.twist.linear.x

    def mondeo2_vel_cb(self, data):
        self.speeds[3] = data.twist.linear.x

    def mkz1_vel_cb(self, data):
        self.speeds[4] = data.twist.linear.x

    def mkz2_vel_cb(self, data):
        self.speeds[5] = data.twist.linear.x

    def fusion1_pose_cb(self, data):
        self.poses[0] = data.pose

    def fusion2_pose_cb(self, data):
        self.poses[1] = data.pose

    def ego_pose_cb(self, data):
        self.poses[2] = data.pose

    def mondeo2_pose_cb(self, data):
        self.poses[3] = data.pose

    def mkz1_pose_cb(self, data):
        self.poses[4] = data.pose

    def mkz2_pose_cb(self, data):
        self.poses[5] = data.pose

    def cur_lane_cb(self, msg):
        self.lane_index = msg.data

    def change_lane_reward_cb(self, msg):
        self.change_lane_reward = msg.data

    def min_dang(self, dang):
        while dang > math.pi:
            dang -= 2.0 * math.pi
        while dang < -math.pi:
            dang += 2.0 * math.pi
        return dang

    # saturation function, values 0 to 1
    def speed_saturate(self, x):
        if x > MAX_SPEED:
            return MAX_SPEED
        if x < 0:
            return 0
        return x

    def phi2quat(self, phi):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(phi / 2.0)
        quaternion.w = math.cos(phi / 2.0)
        return quaternion

    def track_distance(self, path_poses, p1, p2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(p1, p2+1):
            dist += dl(path_poses[p1].pose.pose.position, path_poses[i].pose.pose.position)
            p1 = i
        return dist

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def closest_waypoint_index(self, x, y, poses):
        closest_len = 100000  # large number
        closest_waypoint = 0

        for i in range(len(poses)):
            map_x = poses[i].pose.position.x
            map_y = poses[i].pose.position.y
            dist = self.euclidean_distance(x, y, map_x, map_y)
            if dist < closest_len:
                closest_len = dist
                closest_waypoint = i

        return closest_waypoint

    def next_waypoint(self, x, y, psi, poses):
        closest_waypoint = self.closest_waypoint_index(x, y, poses)

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

        frenet_d = self.euclidean_distance(x_x, x_y, proj_x, proj_y)

        # see if d value is positive or negative by comparing it to a center point

        center_x = 1000 - poses[prev_wp].pose.position.x
        center_y = 2000 - poses[prev_wp].pose.position.y
        center_to_pos = self.euclidean_distance(center_x, center_y, x_x, x_y)
        center_to_ref = self.euclidean_distance(center_x, center_y, proj_x, proj_y)

        if center_to_pos <= center_to_ref:
            frenet_d *= -1

        # calculate s value
        frenet_s = 0
        for i in range(prev_wp):
            frenet_s += self.euclidean_distance(poses[i].pose.position.x, poses[i].pose.position.y, poses[i+1].pose.position.x, poses[i+1].pose.position.y)

        frenet_s += self.euclidean_distance(0, 0, proj_x, proj_y)

        return frenet_s, frenet_d

    def d_to_ilane(self, d):
        if d >= 0 and d < 4:
            ilane = 0
        elif d >= 4 and d < 8:
            ilane = 1
        elif d >=8 and d < 12:
            ilane = 2
        else:
            ilane = None
        return ilane

    def check_relative_postion(self, s, d, sp, dp):
        ilane = self.d_to_ilane(d)
        ilanep = self.d_to_ilane(dp)

        if ilane == -1 or ilanep == -1:
            return None

        if ilanep + 1 == ilane:
            if sp < s:
                return 0
            else:
                return 1
        elif ilanep == ilane:
            if sp < s:
                return -1
            else:
                return 2
        elif ilanep - 1 == ilane:
            if sp < s:
                return 3
            else:
                return 4
        else:
            return -1

    def construct_state(self):
        if self.base_path == None:
            return  self.path_dists + self.speeds + self.lanes + [0], False

        for i in range(6):
            if self.poses[i] == None:
                return self.path_dists + self.speeds + self.lanes + [0], False

        self.path_dists = [80, 80, 80, 80, 80]

        ss = []
        dd = []
        for i in range(6):
            s, d = self.get_frenet(self.poses[i].pose.position.x, self.poses[i].pose.position.y, 0, self.base_path.poses)
            ss.append(s)
            dd.append(d)

        ego_s = ss[2]
        ego_d = dd[2]

        for i in range(2):
            s = ss[i]
            d = dd[i]
            rp = self.check_relative_postion(ego_s, ego_d, s, d)
            if rp == -1:
                continue
            if fabs(s-ego_s) < self.path_dists[rp]:
                self.path_dists[rp] = s - ego_s

        for i in range(3, 6):
            s = ss[i]
            d = dd[i]
            rp = self.check_relative_postion(ego_s, ego_d, s, d)
            if rp == -1:
                continue
            if fabs(s-ego_s) < self.path_dists[rp]:
                self.path_dists[rp] = s - ego_s

        if fabs(self.path_dist[2]) < COLLISON_DIST:
            print("Collision detected!")
            done = True

        for i in range(6):
            if self.poses[i].pose.position.z > 0.5:
                print("The car is turned over!")
                done = True

        self.lanes = [0, 0, 0]
        self.lanes[self.d_to_ilane(ego_d)] = 1

        is_changing_lane = (self.lane_index != self.d_to_ilane(ego_d))

        state = self.path_dists + self.speeds + self.lanes + [is_changing_lane]

        return state, done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):

        state, done = self.construct_state()

        # 27 actions
        speed_cmd = self.speeds[2]
        add_on = [+2.0, +1.5, +1.0, 0.5, 0, -0.5, -1.0, -1.5, -2.0]
        chang_lane_cmds = ["Left", "Keep", "Right"]

        # print("cmd_speed = %f" % cmd_speed)
        speed_cmd = self.speed_saturate(speed_cmd + add_on[action%len(add_on)])
        chang_lane_cmd = chang_lane_cmds[action/len(add_on)]
        self.cruise_speed_pub.publish(speed_cmd)
        self.change_lane_pub.publish(chang_lane_cmd)

        # 27 actions
        reward = 0
        if not done:
            mid = len(add_on)/2
            reward += -abs((action%len(add_on))-mid)*5 + mid*5
            mid = len(chang_lane_cmds)/2
            reward += -abs((action/len(add_on))-mid)*10
        else:
            reward += -10000
            self.travel_dist = 0

        if self.prev_ego_pose == None:
            acc_dist = 0
        else:
            x0, y0 = self.prev_ego_pose.pose.position.x, self.prev_ego_pose.pose.position.y
            x1, y1 = self.poses[2].pose.position.x, self.poses[2].pose.position.y
            acc_dist = self.euclidean_distance(x0, y0, x1, y1)
        self.travel_dist += acc_dist
        self.prev_ego_pose = self.poses[2]

        # speed
        reward += 1.0 * (MAX_SPEED - fabs(MAX_SPEED - self.speeds[2]))

        # change lane reward
        reward += self.change_lane_reward

        # by acc_dist
        reward += 10 * acc_dist

        if self.travel_dist >= 1000:
            print("Safely finishing! :D")
            done = True
            self.travel_dist = 0

        return np.asarray(state), reward, done, {}

    def _reset(self):
        '''
        Resets the state of the environment and returns an initial observation.
        '''

        # ************************************************
        # Pause simulation to make observation
        # ************************************************
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        # ************************************************
        # define initial pose for later use
        # ************************************************
        init_pose0 = Pose()
        init_pose0.position.x = -10.0
        init_pose0.position.y = -2.0
        init_pose0.position.z = 0.0
        init_pose0.orientation.x = 0.0
        init_pose0.orientation.y = 0.0
        init_pose0.orientation.z = 0.0
        init_pose0.orientation.w = 0.0

        init_pose1 = Pose()
        init_pose1.position.x = 10.0
        init_pose1.position.y = -2.0
        init_pose1.position.z = 0.0
        init_pose1.orientation.x = 0.0
        init_pose1.orientation.y = 0.0
        init_pose1.orientation.z = 0.0
        init_pose1.orientation.w = 0.0

        init_pose2 = Pose()
        init_pose2.position.x = -10.0
        init_pose2.position.y = -6.0
        init_pose2.position.z = 0.0
        init_pose2.orientation.x = 0.0
        init_pose2.orientation.y = 0.0
        init_pose2.orientation.z = 0.0
        init_pose2.orientation.w = 0.0

        init_pose3 = Pose()
        init_pose3.position.x = 10.0
        init_pose3.position.y = -6.0
        init_pose3.position.z = 0.0
        init_pose3.orientation.x = 0.0
        init_pose3.orientation.y = 0.0
        init_pose3.orientation.z = 0.0
        init_pose3.orientation.w = 0.0

        init_pose4 = Pose()
        init_pose4.position.x = -10.0
        init_pose4.position.y = -10.0
        init_pose4.position.z = 0.0
        init_pose4.orientation.x = 0.0
        init_pose4.orientation.y = 0.0
        init_pose4.orientation.z = 0.0
        init_pose4.orientation.w = 0.0

        init_pose5 = Pose()
        init_pose5.position.x = 10.0
        init_pose5.position.y = -10.0
        init_pose5.position.z = 0.0
        init_pose5.orientation.x = 0.0
        init_pose5.orientation.y = 0.0
        init_pose5.orientation.z = 0.0
        init_pose5.orientation.w = 0.0

        # ************************************************
        # set initial model state
        # ************************************************
        model_state1 = ModelState()
        model_state1.model_name = "fusion1"
        model_state1.pose = init_pose0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state1)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state2 = ModelState()
        model_state2.model_name = "fusion2"
        model_state2.pose = init_pose1

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state2)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state3 = ModelState()
        model_state3.model_name = "ego"
        model_state3.pose = init_pose2

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state3)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state4 = ModelState()
        model_state4.model_name = "mondeo2"
        model_state4.pose = init_pose3

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state4)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state5 = ModelState()
        model_state5.model_name = "mkz1"
        model_state5.pose = init_pose4

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state5)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state6 = ModelState()
        model_state6.model_name = "mkz2"
        model_state6.pose = init_pose5

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state6)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        # ************************************************
        # Unpause simulation to make observation
        # ************************************************
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed")

        # ************************************************
        # construct state
        # ************************************************
        state, done = self.construct_state()

        return np.asarray(state)
