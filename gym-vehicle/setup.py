from setuptools import setup
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-vehicle'))

setup(name='gym-vehicle',
      version='1.0.0',
      install_requires=['gym>=0.2.3'],
      description='The OpenAI Gym for robotics: A toolkit for developing and comparing your reinforcement learning agents using Gazebo and ROS.',
      url='https://github.com/xpharry/gym_vehicle',
      author='Peng Xu',
      package_data={'gym-vehicle': ['envs/assets/launch/*.launch', 'envs/assets/worlds/*']},
)
