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

from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates

MAX_SPEED = 22.35  # m/sec; tune this

class GazeboTrackCatvehicleLidarNnEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboTrackCatvehicleLidar_v0.launch")

        self.speed = 0
        self.path_dist = 0
        self.travel_dist = 0
        self.prev_pose = Pose()
        self.pose = Pose()

        rospy.Subscriber('/catvehicle2/vel', Twist, self.velCallback)
        rospy.Subscriber('/catvehicle2/odom', Odometry, self.odomCallback)

        self.vel_pub = rospy.Publisher('/catvehicle2/des_speed', Float64, queue_size=5)
        self.reset_sim_pub1 = rospy.Publisher('/catvehicle1/reset_sim', String, queue_size=10)
        self.reset_sim_pub2 = rospy.Publisher('/catvehicle2/reset_sim', String, queue_size=10)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def velCallback(self, data):
        self.speed = data.linear.x

    def odomCallback(self, data):
        self.pose = data.pose.pose

    def min_dang(self, dang):
        while dang > math.pi:
            dang -= 2.0 * math.pi
        while dang < -math.pi:
            dang += 2.0 * math.pi
        return dang

    # saturation function, values 0 to 1
    def saturate(self, x):
        if x > MAX_SPEED:
            return 1.0
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

    def generate_track_path(self):
        ra = 36.6
        nedges = 301
        medges = int(nedges * math.pi)

        path = []

        # straight line 1
        for i in range(nedges):
            # point i
            pose = Pose()
            pose.position.x = i * 2 * ra / nedges - ra
            pose.position.y = 0.0
            pose.position.z = 0.0

            theta = 0.0
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # half circle 1
        phi0 = -math.pi / 2
        for i in range(int(medges / 2)):
            # point i
            pose = Pose()
            phi = self.min_dang(phi0 + 2.0 * math.pi / medges * i)
            pose.position.x = ra * math.cos(phi) + ra
            pose.position.y = ra * math.sin(phi) + ra
            pose.position.z = 0.0

            theta = self.min_dang(phi + math.pi / 2.0)
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # straight line 2
        for i in range(nedges):
            # point i
            pose = Pose()
            pose.position.x = - i * 2 * ra / nedges + ra
            pose.position.y = 2.0 * ra
            pose.position.z = 0.0

            theta = math.pi
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # half circle 2
        phi0 = -math.pi / 2
        for i in range(int(medges/2), medges):
            # point i
            pose = Pose()
            phi = self.min_dang(phi0 + 2.0 * math.pi / medges * i)
            pose.position.x = ra * math.cos(phi) - ra
            pose.position.y = ra * math.sin(phi) - ra
            pose.position.z = 0.0

            theta = self.min_dang(phi + math.pi / 2.0)
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        print("=============== Status Bar ==================")
        print("path generated! length = %d" % len(path))
        return path

    def compute_dist(self, a, b):
        x1 = a.position.x
        y1 = a.position.y
        x2 = b.position.x
        y2 = b.position.y
        return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

    def find_index(self, p, path):
        min_dist = 100000.0
        idx = 0
        for i in range(len(path)):
            dist = self.compute_dist(p, path[i])
            # print(path[i].position.x, path[i].position.y)
            if dist < min_dist:
                # print("found index = %d" % i)
                idx = i
                min_dist = dist
        return idx

    def calculate_track_dist(self, p1, p2, path):
        unit_dist = self.compute_dist(path[0], path[1])
        index1 = self.find_index(p1, path)
        index2 = self.find_index(p2, path)
        num_units = index1 - index2
        if num_units < 0:
            num_units = num_units + len(path)
        track_dist = num_units * unit_dist
        print("index 1 = %d; index 2 = %d" % (index1, index2))
        # print(track_dist)
        return track_dist

    def calculate_state(self, data):
        n_models = len(data.name)
        imodel1 = 0
        imodel2 = 0
        for i in range(n_models):
            model_name = data.name[i]
            if model_name == "catvehicle1":
                # rospy.loginfo("found match: mobot is model %d", i)
                imodel1 = i
            if model_name == "catvehicle2":
                # rospy.loginfo("found match: mobot is model %d", i)
                imodel2 = i

        p1 = data.pose[imodel1]
        p2 = data.pose[imodel2]
        v1 = data.twist[imodel1].linear.x
        v2 = data.twist[imodel2].linear.x
        path = self.generate_track_path()

        min_dist = 4.0
        done = False
        self.path_dist = self.calculate_track_dist(p1, p2, path)
        print("path dist = %f " % self.path_dist)

        if self.path_dist < min_dist:
            print("Collision detected!")
            done = True

        if self.pose.position.z > 0.5:
            print("The car is turned over!")
            done = True

        state = [self.path_dist, v1 - v2]
        return state, done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed")

        '''# 21 actions
        max_ang_speed = 0.3
        ang_vel = (action-10)*max_ang_speed*0.1 #from (-0.33 to + 0.33)

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.2
        vel_cmd.angular.z = ang_vel
        self.vel_pub.publish(vel_cmd)'''

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        state, done = self.calculate_state(data)

        # 3 actions
        cmd_speed = self.speed
        # print("cmd_speed = %f" % cmd_speed)
        if action == 0:  # stay current velocity
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 1:  # slight accelerate
            cmd_speed += 0.5
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 2:  # ordinary accelerate
            cmd_speed += 1
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 3:  # heavy accelerate
            cmd_speed += 2
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 4:  # slight accelerate
            cmd_speed -= 0.5
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 5:  # ordinary decelerate
            cmd_speed -= 1
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)
        elif action == 6:  # heavy decelerate
            cmd_speed -= 2
            cmd_speed = self.saturate(cmd_speed)
            self.vel_pub.publish(cmd_speed)

        print("travel distance = %f" % self.travel_dist)

        # if not done:

        #     reward = round(15*(max_ang_speed - abs(ang_vel) + 0.0335), 2)
        #     # print ("Action : "+str(action)+" Ang_vel : "+str(ang_vel)+" reward="+str(reward))
        # else:
        #     reward = -200

        # 7 actions
        # stay current velocity reward = 5, change velocity reward = 0.5
        if not done:
            if action == 0:  # stay current speed
                reward = 10
            elif action == 1:
                reward = 5
            elif action == 2:
                reward = 0
            elif action == 3:
                reward = -5
            elif action == 4:
                reward = 5
            elif action == 5:
                reward = 0
            else:
                reward = -5
        else:
            reward = -10000
            self.travel_dist = 0

        acc_dist = self.compute_dist(self.prev_pose, self.pose)
        self.travel_dist += acc_dist
        self.prev_pose = self.pose

        # by speed and relative speed
        if cmd_speed > MAX_SPEED*0.8:
            speed_reward = -15 * (cmd_speed - MAX_SPEED*0.8)
        else:
            speed_reward = -5 * (MAX_SPEED*0.8 - cmd_speed)
        reward -= speed_reward

        # by path dist
        path_dist_reward = 12.5
        if self.path_dist < 40:
            path_dist_reward = 100 * math.pow((self.path_dist - 20) / 40, 3)
        reward += path_dist_reward

        # by acc_dist
        reward += 10 * acc_dist

        if self.travel_dist >= 376:
            print("Safely finishing! :D")
            done = True
            self.travel_dist = 0

        return np.asarray(state), reward, done, {}

    def _reset(self):
        # # Resets the state of the environment and returns an initial observation.
        # rospy.wait_for_service('/gazebo/reset_simulation')
        # try:
        #     #reset_proxy.call()
        #     self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/reset_simulation service call failed")

        # define initial pose for later use
        pose1 = Pose()
        pose1.position.x = 0.0
        pose1.position.y = 0.0
        pose1.position.z = -0.30
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 0.0

        # set initial model state
        model_state1 = ModelState()
        model_state1.model_name = "catvehicle1"
        model_state1.pose = pose1

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state1)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        # define initial pose for later use
        pose2 = Pose()
        pose2.position.x = -20.0
        pose2.position.y = 0.0
        pose2.position.z = -0.30
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 0.0

        # set initial model state
        model_state2 = ModelState()
        model_state2.model_name = "catvehicle2"
        model_state2.pose = pose2

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(model_state2)
            print(ret.status_message)
        except rospy.ServiceException as e:
            print("Service \'set_model_state\' call failed: %s" % e)

        msg = String()
        msg.data = "123"
        self.reset_sim_pub1.publish(msg)
        self.reset_sim_pub2.publish(msg)

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        state, done = self.calculate_state(data)

        return np.asarray(state)
