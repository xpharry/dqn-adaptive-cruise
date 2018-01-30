
## Table of Contents
- [Installation](#installation)
	- [Ubuntu 14.04 (**DOING**)](#ubuntu-1404)

## Installation

### Ubuntu 14.04
Basic requirements:
- ROS Indigo (`/rosversion: 1.12.7`)
- Gazebo 2.2.6
- Python 2.7
- OpenAI gym

#### ROS Indigo

Install the Robot Operating System via:

**ros-indigo-desktop-full** is the only recommended installation.

- Ubuntu: http://wiki.ros.org/indigo/Installation/Ubuntu
- Others: http://wiki.ros.org/indigo/Installation

#### Gym Vehicle Pip

```bash
git clone https://github.com/xpharry/gym-vehicle
cd gym-vehicle
sudo pip install -e .
```

#### Additional dependencies

