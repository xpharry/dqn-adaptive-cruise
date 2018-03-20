# gym-vehicle

## Usage

Build and install gym-gazebo
In the root directory of the repository:

```
sudo pip install -e .
```

### Display the simulation

To see what's going on in Gazebo during a simulation, simply run gazebo client:

```bash
gzclient
```

### Running an environment

- Load the environment variables corresponding to the robot you want to launch.

```bash
bash entrypoint.bash
```

Note: all the setup scripts are available in `gym_vehicle/envs/installation`

- Run any of the examples available in `experiments/`. E.g.:

```bash
cd experiments/vehicle
python standard_track_multi_vehicle_lidar_dqn.py
```

### Display reward plot

Display a graph showing the current reward history by running the following script:

```bash
cd examples/utilities
python display_plot.py
```

HINT: use `--help` flag for more options.

## Killing background processes

Sometimes, after ending or killing the simulation gzserver and rosmaster stay on the background, make sure you end them before starting new tests.

We recommend creating an alias to kill those processes.

echo "alias killgazebogym='killall -9 roscore roslaunch rosmaster rosout gzserver nodelet mkz_spawner robot_state_publisher dbw_node gazebo_state_pub pure_pursuit twist_controller rviz gzclient python'" >> ~/.bashrc