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

DISPLAY_STATE = False

MAX_SPEED = 22.35  # m/sec; tune this
COLLISON_DIST = 10 # m
INIT_LANE_INDEX = 1
LAPS = 3

class GazeboCircletrack2VehicleAutoEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboCircletrack2VehicleAuto_v0.launch")

        self.base_path = None
        self.speeds = [0, 0, 0, 0, 0]
        self.poses = [None, None, None, None, None]
        self.lane_index = INIT_LANE_INDEX
        self.lanes = [0, 0, 0]
        self.travel_dist = 0
        self.travel_time = 0
        self.time_stamp = None
        self.prev_ego_pose = None
        self.prev_speed = 10
        self.time_steps = 0
        self.change_lane_reward = 0

        rospy.Subscriber('/base_path', Path, self.base_path_cb)

        rospy.Subscriber('/ego/twist', TwistStamped, self.ego_vel_cb)
        rospy.Subscriber('/fusion1/twist', TwistStamped, self.fusion1_vel_cb)
        rospy.Subscriber('/fusion2/twist', TwistStamped, self.fusion2_vel_cb)
        rospy.Subscriber('/mondeo1/twist', TwistStamped, self.mondeo1_vel_cb)
        rospy.Subscriber('/mondeo2/twist', TwistStamped, self.mondeo2_vel_cb)

        rospy.Subscriber('/ego/current_pose', PoseStamped, self.ego_pose_cb)
        rospy.Subscriber('/fusion1/current_pose', PoseStamped, self.fusion1_pose_cb)
        rospy.Subscriber('/fusion2/current_pose', PoseStamped, self.fusion2_pose_cb)
        rospy.Subscriber('/mondeo1/current_pose', PoseStamped, self.mondeo1_pose_cb)
        rospy.Subscriber('/mondeo2/current_pose', PoseStamped, self.mondeo2_pose_cb)

        rospy.Subscriber('/ego/current_lane', Int32, self.cur_lane_cb)
        rospy.Subscriber('/ego/chang_lane_reward', Int32, self.change_lane_reward_cb)

        self.cruise_speed_pub = rospy.Publisher('/ego/cruise_speed', Float64, queue_size=5)
        self.change_lane_pub = rospy.Publisher('/ego/change_lane', String, queue_size=5)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.action_space = spaces.Discrete(21)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def base_path_cb(self, msg):
        """ Store the given map """
        self.base_path = msg

    def ego_vel_cb(self, data):
        self.speeds[0] = data.twist.linear.x

    def fusion1_vel_cb(self, data):
        self.speeds[1] = data.twist.linear.x

    def fusion2_vel_cb(self, data):
        self.speeds[2] = data.twist.linear.x

    def mondeo1_vel_cb(self, data):
        self.speeds[3] = data.twist.linear.x

    def mondeo2_vel_cb(self, data):
        self.speeds[4] = data.twist.linear.x

    def ego_pose_cb(self, data):
        self.poses[0] = data

    def fusion1_pose_cb(self, data):
        self.poses[1] = data

    def fusion2_pose_cb(self, data):
        self.poses[2] = data

    def mondeo1_pose_cb(self, data):
        self.poses[3] = data

    def mondeo2_pose_cb(self, data):
        self.poses[4] = data

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

    def quat2phi(self, quat):
        quat_z = quat.z
        quat_w = quat.w
        phi = 2.0 * math.atan2(quat_z, quat_w)
        return phi

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
        if p2 < p1:
            p2 += len(path_poses)
        for i in range(p1, p2+1):
            dist += dl(path_poses[p1%len(path_poses)].pose.position, path_poses[i%len(path_poses)].pose.position)
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

        angle = abs(psi-heading)
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

        center_x = 0 - poses[prev_wp].pose.position.x
        center_y = 36.6 - poses[prev_wp].pose.position.y
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
        else:
            # print("error lane index: d = %f" % d)
            ilane = None
        return ilane

    def compute_delta_s(self, pose1, pose2):
        p1 = self.closest_waypoint_index(pose1.pose.position.x, pose1.pose.position.y, self.base_path.poses)
        p2 = self.closest_waypoint_index(pose2.pose.position.x, pose2.pose.position.y, self.base_path.poses)
        dist1 = self.track_distance(self.base_path.poses, p1, p2)
        dist2 = self.track_distance(self.base_path.poses, p2, p1)
        if dist1 < dist2:
            return dist1
        else:
            return -dist2

    def check_relative_postion(self, delta_s, d1, d2):
        ilane1 = self.d_to_ilane(d1)
        ilane2 = self.d_to_ilane(d2)

        if ilane1 is None or ilane2 is None:
            return -1

        if ilane1 == -1 or ilane2 == -1:
            return None

        if ilane2 + 1 == ilane1:
            if delta_s < 0:
                return 0
            else:
                return 1
        elif ilane2 == ilane1:
            if delta_s < 0:
                return 2
            else:
                return 3
        elif ilane2 - 1 == ilane1:
            if delta_s < 0:
                return 4
            else:
                return 5
        else:
            return -1

    def construct_state(self):
        cmp_dists = [80, 80, 80, 80, 80, 80]
        cmp_speeds = [0, 20, 0, 20, 0, 20]

        ego_d = 6.0

        if self.base_path == None:
            return  [ego_d] + cmp_dists + [self.speeds[0]] + cmp_speeds, False

        for i in range(5):
            if self.poses[i] == None:
                return [ego_d] + cmp_dists + [self.speeds[0]] + cmp_speeds, False

        ss = []
        dd = []
        for i in range(5):
            x = self.poses[i].pose.position.x
            y = self.poses[i].pose.position.y
            psi = self.quat2phi(self.poses[i].pose.orientation)
            s, d = self.get_frenet(x, y, psi, self.base_path.poses)
            # if i == 2:
            #     print("s = %f, d = %f" %(s, d))
            ss.append(s)
            dd.append(d)

        ego_s = ss[0]
        ego_d = dd[0]

        if ego_d < 0 or ego_d >= 8:
            return [ego_d] + cmp_dists + [self.speeds[0]] + cmp_speeds, True

        for i in range(1, 5):
            s = ss[i]
            d = dd[i]
            delta_s = self.compute_delta_s(self.poses[0], self.poses[i])
            rp = self.check_relative_postion(delta_s, ego_d, d)
            if rp == -1:
                continue
            if abs(delta_s) < abs(cmp_dists[rp]):
                cmp_dists[rp] = abs(delta_s)
                cmp_speeds[rp] = self.speeds[i]

        done = False

        if self.d_to_ilane(ego_d) == 0:
            cmp_dists[0] = 5.0
            cmp_dists[1] = 5.0
            cmp_speeds[0] = 0.0
            cmp_speeds[1] = 0.0
        elif self.d_to_ilane(ego_d) == 1:
            cmp_dists[4] = 5.0
            cmp_dists[5] = 5.0
            cmp_speeds[4] = 0.0
            cmp_speeds[5] = 0.0
        else:
            pass

        if DISPLAY_STATE:
            print("\n")
            print("|----------------- Current State ---------------|")
            print("| Compared Dist:                                |")
            print("| %f \t| %f \t| %f \t|" % (cmp_dists[1], cmp_dists[3], cmp_dists[5]))
            print("|---------------|----- Ego -----|---------------|")
            print("| %f \t| %f \t| %f \t|" % (cmp_dists[0], cmp_dists[2], cmp_dists[4]))
            print("| Compared Speed:                               |")
            print("| %f \t| %f \t| %f \t|" % (cmp_speeds[1], cmp_speeds[3], cmp_speeds[5]))
            print("|---------------| %f \t|---------------|" % (self.speeds[0]))
            print("| %f \t| %f \t| %f \t|" % (cmp_speeds[0], cmp_speeds[2], cmp_speeds[4]))
            print("|-----------------------------------------------|")
            print("| Current Lane: %d \t\t\t|" % (self.d_to_ilane(ego_d)))
            print("| Travel Distance: %f \t\t\t|" % (self.travel_dist))
            print("| Travel Time: %f \t\t\t|" % (self.travel_time))
            print("| Current Speed: %f \t\t\t|" % (self.speeds[0]))
            print("| Average Speed: %f \t\t\t|" % (0 if self.travel_time == 0 else self.travel_dist/self.travel_time))
            print("|-----------------------------------------------|")

        if abs(cmp_dists[2]) < COLLISON_DIST or abs(cmp_dists[3]) < COLLISON_DIST:
            print("Collision detected!")
            done = True
        # elif self.time_steps > 5  and self.speeds[0] < 2.0:
        #     print("Too slow ...")
        #     done = True

        for i in range(5):
            if self.poses[i].pose.position.z > 0.5:
                print("The car is turned over!")
                done = True

        self.lanes = [0, 0, 0]
        # self.lanes[self.d_to_ilane(ego_d)] = 1

        is_changing_lane = (self.lane_index != self.d_to_ilane(ego_d))

        state = [ego_d] + cmp_dists + [self.speeds[0]] + cmp_speeds

        return state, done

    def action_names(self, action):
        i = action%5
        j = action/5
        action_move_forward = ["Accelerate +2.0 m/s",
                               "Accelerate +1.0 m/s",
                               "Keep current speed",
                               "Decelerate -1.0 m/s",
                               "Decelerate -2.0 m/s"]
        action_change_lane = ["Change to Left",
                              "Keep Lane",
                              "Change to Right"]
        return action_move_forward[i] + " & " + action_change_lane[j]

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):

        state, done = self.construct_state()

        # 27 actions
        speed_cmd = self.speeds[0]
        # speed_cmd = self.prev_speed
        chang_lane_cmd = None
        add_on = [+2.0, +1.0, 0, -1.0, -2.0]
        chang_lane_cmds = ["Left", "Keep", "Right"]

        speed_cmd = self.speed_saturate(speed_cmd + add_on[action%len(add_on)])
        chang_lane_cmd = chang_lane_cmds[action/len(add_on)]
        self.cruise_speed_pub.publish(speed_cmd)
        self.change_lane_pub.publish(chang_lane_cmd)

        # if action == 0:
        #     self.cruise_speed_pub.publish(speed_cmd)
        # elif action == 1:
        #     speed_cmd = self.speed_saturate(speed_cmd + 1.0)
        #     self.cruise_speed_pub.publish(speed_cmd)
        # elif action == 2:
        #     speed_cmd = self.speed_saturate(speed_cmd - 1.0)
        #     self.cruise_speed_pub.publish(speed_cmd)
        # elif action == 3:
        #     chang_lane_cmd = "Left"
        #     self.change_lane_pub.publish(chang_lane_cmd)
        # else:
        #     chang_lane_cmd = "Right"
        #     self.change_lane_pub.publish(chang_lane_cmd)

        # self.prev_speed = speed_cmd

        reward = 0

        if self.d_to_ilane(state[0]) == 0 and chang_lane_cmd == "Left":
            done = True
        if self.d_to_ilane(state[0]) == 1 and chang_lane_cmd == "Right":
            done = True

        if done:
            # reward += -10000
            self.travel_dist = 0
            self.travel_time = 0
            self.prev_ego_pose = None
            self.time_steps = 0
            return np.asarray(state), reward, done, {}

        reward += -(action%len(add_on) - len(add_on)/2) * 0.2

        if chang_lane_cmd == "Left" or chang_lane_cmd == "Right":
            reward += -0.5

        if self.prev_ego_pose == None:
            acc_dist = 0
        else:
            x0, y0 = self.prev_ego_pose.pose.position.x, self.prev_ego_pose.pose.position.y
            x1, y1 = self.poses[0].pose.position.x, self.poses[0].pose.position.y
            acc_dist = self.euclidean_distance(x0, y0, x1, y1)
        self.travel_dist += acc_dist
        self.prev_ego_pose = self.poses[0]

        if self.time_stamp is None:
            self.travel_time = 0
        else:
            self.travel_time += rospy.get_time() - self.time_stamp
        self.time_stamp = rospy.get_time()

        # speed
        # reward += 10.0 * (MAX_SPEED - abs(MAX_SPEED - self.speeds[0]))

        # change lane reward
        # reward += self.change_lane_reward

        reward += -abs(state[3]-state[4]) / (state[3]+state[4])

        # by acc_dist
        reward += 1.0 # * acc_dist
        self.time_steps += 1

        if DISPLAY_STATE:
            print("| Action: %s\t|" % action)
            print("| Reward: %f \t\t\t\t|" % reward)
            print("|-----------------------------------------------|")

        # if self.travel_time >= 25.0 * LAPS:
        #     print("Time Out! :(")
        #     reward += -10000
        #     done = True
        #     self.travel_dist = 0
        #     self.travel_time = 0

        # if self.travel_dist >= 280 * LAPS:
        #     print("Safely finishing! :)")
        #     reward += 5000
        #     done = True
        #     self.travel_dist = 0
        #     self.travel_time = 0

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
        init_pose0.position.x = 42.6
        init_pose0.position.y = 36.6
        init_pose0.position.z = 0.0
        quat0 = self.phi2quat(1.57)
        init_pose0.orientation = quat0

        init_pose1 = Pose()
        init_pose1.position.x = 0.0
        init_pose1.position.y = -2.0
        init_pose1.position.z = 0.0
        quat1 = self.phi2quat(0)
        init_pose1.orientation = quat1

        init_pose2 = Pose()
        init_pose2.position.x = 0.0
        init_pose2.position.y = 75.2
        init_pose2.position.z = 0.0
        quat2 = self.phi2quat(math.pi)
        init_pose2.orientation = quat2

        init_pose3 = Pose()
        init_pose3.position.x = 0.0
        init_pose3.position.y = -6.0
        init_pose3.position.z = 0.0
        quat3 = self.phi2quat(0.0)
        init_pose3.orientation = quat3

        init_pose4 = Pose()
        init_pose4.position.x = 0.0
        init_pose4.position.y = 79.2
        init_pose4.position.z = 0.0
        quat4 = self.phi2quat(math.pi)
        init_pose4.orientation = quat4

        # ************************************************
        # set initial model state
        # ************************************************
        model_state0 = ModelState()
        model_state0.model_name = "ego"
        model_state0.pose = init_pose0
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state0)
            # print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state1 = ModelState()
        model_state1.model_name = "fusion1"
        model_state1.pose = init_pose1
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state1)
            # print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state2 = ModelState()
        model_state2.model_name = "fusion2"
        model_state2.pose = init_pose2
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state2)
            # print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state3 = ModelState()
        model_state3.model_name = "mondeo1"
        model_state3.pose = init_pose3
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state3)
            # print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        model_state4 = ModelState()
        model_state4.model_name = "mondeo2"
        model_state4.pose = init_pose4
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state4)
            # print(ret.status_message)
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
