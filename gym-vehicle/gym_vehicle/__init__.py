import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Catvehicle envs
register(
    id='GazeboCartPole-v0',
    entry_point='gym_vehicle.envs.gazebo_cartpole:GazeboCartPolev0Env',
    # More arguments here
)
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
    id='GazeboCircletrack1VehicleAcc-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrack1VehicleAccEnv',
    # More arguments here
)
register(
    id='GazeboCircletrack1VehicleAcc-v1',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrackVehicleAccEnv',
    # More arguments here
)
register(
    id='GazeboCircletrack2VehicleLcc-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrack2VehicleLccEnv',
    # More arguments here
)
register(
    id='GazeboCircletrack2VehicleAuto-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrack2VehicleAutoEnv',
    # More arguments here
)
register(
    id='GazeboCircletrack3VehicleLcc-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrack3VehicleLccEnv',
    # More arguments here
)
register(
    id='GazeboCircletrack3VehicleAuto-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboCircletrack3VehicleAutoEnv',
    # More arguments here
)
register(
    id='GazeboStandardtrackVehicleLcc-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboStandardtrackVehicleLccEnv',
    # More arguments here
)
register(
    id='GazeboStandardtrackVehicleAuto-v0',
    entry_point='gym_vehicle.envs.vehicle:GazeboStandardtrackVehicleAutoEnv',
    # More arguments here
)

