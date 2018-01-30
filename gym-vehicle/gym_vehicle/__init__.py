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