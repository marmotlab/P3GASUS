import numpy as np

class DriverParameters:
    SCENARIO = 0      # 0: MAPF, 1:Traffic
    
    VIRTUAL_TO_REAL_ROBOT_MAPPING = {}
    # VIRTUAL_TO_REAL_ROBOT_MAPPING = {0:1,1:6,2:8,3:9, 4:11,5:2,6:7,7:5}
    
    HYBRID_ROBOT_COUNT = 8
    GROUP_COUNT = 1
    
    RESOLUTION = 0.3
    OPTITRACK_TO_MAP_SHIFT= [0, 0]
    
    VRPN_IP = "10.42.0.25"
    ROS_MASTER_IP = "10.42.0.1"
    REAL_ROBOT_IP = "10.42.0."
    DEBUG = False
    REAL_WORLD_SIZE = (10,10)
    
    # VELOCITY_ERROR = np.random.uniform(-0.3, 0.3, HYBRID_ROBOT_COUNT)
    VELOCITY_ERROR = np.zeros(HYBRID_ROBOT_COUNT)

    assert HYBRID_ROBOT_COUNT>=GROUP_COUNT

class MAPFParameters:

    WORLD_SIZE = (1e8,1e8)
    WORLD_SIZE = DriverParameters.REAL_WORLD_SIZE

    INTERACTIVE_GOAL_NUM = 0
    PARK = False

    IMITATE = False
    IMITATE_FOLDER = "/scenarios/MAPF/75_robots/"        
    # IMITATE_FOLDER = "/scenarios/MAPF/10_FSP/"        
    IMITATION_LIST = np.arange(0, DriverParameters.HYBRID_ROBOT_COUNT)
    IMITATION_LENGTH = 1e8
    
    SAFETY_DISTANCE = 0.3
    
    WORLD = None
    STARTS = None
    
    VIRTUAL_FAST_VEL = 0.005 # m/s
    VIRTUAL_SLOW_VEL = 0.0025 # m/s
    VIRTUAL_FAR_THRESHOLD=0.01 # meters
    VIRTUAL_REACH_THRESHOLD=0.001 # meters
    
    REAL_FAST_VEL = 0.25 # m/s
    REAL_SLOW_VEL = 0.1 # m/s
    REAL_FAR_THRESHOLD=0.1 # meters
    REAL_REACH_THRESHOLD=0.01 # meters

    ANGULAR_REACH_THRESHOLD = 0.01 # radians
    REAL_ANGULAR_VEL = 0.5
    
class TrafficParameters():
    ORIGIN = [75.25, 5.25]

    FUTURE_TASKS = 15
    FAR_THRESHOLD = 0.1
    REACH_THRESHOLD=0.05
    
    VIRTUAL_MAX_VEL = 0.05
    VIRTUAL_MIN_VEL = 0.01

    REAL_FAST_VEL = 0.25 # m/s
    REAL_SLOW_VEL = 0.025 # m/s
    
    SPEED_MULTIPLIER = 0.1
    
    ANG_THRESHOLD = np.pi/3

    MAX_ANG_VEL = 0.3
    MIN_ANG_VEL = 0.0001


    