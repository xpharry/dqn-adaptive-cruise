import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Catvehicle envs
register(
    id='GazeboCircuitLargeCatvehicleLidar-v0',
    entry_point='gym_vehicle.envs.catvehicle:GazeboCircuitLargeCatvehicleLidarEnv',
    # More arguments here
)
register(
    id='GazeboCircuitLargeCatvehicleLidarNn-v0',
    entry_point='gym_vehicle.envs.catvehicle:GazeboCircuitLargeCatvehicleLidarNnEnv',
    # More arguments here
)
register(
    id='GazeboTrackCatvehicleLidar-v0',
    entry_point='gym_vehicle.envs.catvehicle:GazeboTrackCatvehicleLidarEnv',
    # More arguments here
)
register(
    id='GazeboTrackCatvehicleLidarNn-v0',
    entry_point='gym_vehicle.envs.catvehicle:GazeboTrackCatvehicleLidarNnEnv',
    # More arguments here
)
register(
    id='GazeboCircleTrackMultiVehicleLidarNn-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircleTrackMultiVehicleLidarNnEnv',
    # More arguments here
)
register(
    id='GazeboThreeLaneCircleTrackMultiVehicleLidarNn-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboThreeLaneCircleTrackMultiVehicleLidarNnEnv',
    # More arguments here
)
register(
    id='GazeboStandardTrackMultiVehicleLidarNn-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboStandardTrackMultiVehicleLidarNnEnv',
    # More arguments here
)
register(
    id='GazeboCartPole-v0',
    entry_point='gym_vehicle.envs.gazebo_cartpole:GazeboCartPolev0Env',
    # More arguments here
)
