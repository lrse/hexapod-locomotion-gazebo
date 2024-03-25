import os
import uuid
import sys
import time
import rospy
import socket
import struct
import random
import numpy as np
import matplotlib.pyplot as plt
import math
import gymnasium as gym
from loguru import logger
from itertools import chain
from collections import deque
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64, Float64MultiArray
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import SetModelState, DeleteModel
from geometry_msgs.msg import Pose  
from terrain_generator import octaves_perlin, parser_model_sdf, stairs_generator
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, Point

#-----------------------------------------#
# Constants defined for the especific robot:
LEGS = ['rf', 'rm', 'rr', 'lf', 'lm', 'lr'] 
FREQ_SAMPLE = 50
F0 = 1.25
# SCALE_DELTA_R = 2.43 # dr \in [-SCALE_DELTA_R cm, SCALE_DELTA_R cm]
# SCALE_FI = 50 # fi \in [-SCALE_FI, SCALE_FI]

# Available Velocity Commands:
COMMAND_LINEAL = [np.array([0,0],dtype = np.float64),
                  np.array([1,0],dtype = np.float64),
                #   np.array([0,1],dtype = np.float64),
                #   np.array([0,-1],dtype = np.float64),
                  np.array([-1,0],dtype = np.float64)]
COMMAND_ANGULAR = [0, 1, -1]

LINEAL_VEL_THRESHOLD = 0.1 # This threshold represents the maximum speed reachable on flat terrain with a reference controller.
ANGULAR_VEL_THRESHOLD = 0.1 # Value from the publication: 0.6

# Values needed for kinematic calculations (used for random initial condition):
LC = 5.053  # coxa lenght (cm)
LF = 6.828  # femur lenght (cm)
LT = 12.998 # tibia lenght (cm)
COXA_ORIENTATIONS = {'rf': -np.pi/4, 'rm': -np.pi/2, 'rr': -3*np.pi/4, 'lf': np.pi/4, 'lm': np.pi/2, 'lr': 3*np.pi/4}
# Tolerance for 0 value of forces:
# FORCE_TOLERANCE = 0.001
#-----------------------------------------#

class PhantomxAtc(gym.Env):

    """ This is the PhantomX environment that follows gym interface 
    
        Adaptive terrain curriculum (ATC) is a curriculum learning method that
        adaptively generates terrains for training. ATC is based on the idea that
        the difficulty of the terrain should be gradually increased as the robot
        learns. """

    def __init__(self, max_timesteps_per_episode=1000):
        # Inherit from gym.Env
        super(PhantomxAtc,self).__init__()
        # Initialize the node
        rospy.init_node('enviroment_node')
        ###############################################################################
        # Create a socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server
        server_address = ('localhost', 12345)  
        self.client_socket.connect(server_address)
        # wait to stablish the connection
        time.sleep(0.5)
        # Rotation velocity of the base link (+1: clocwise, -1: counterclockwise, 0: No_rotation)
        
        self.vel_base            = np.array([0,0,0], dtype=np.float64)     # linear velocity of the base
        self.vel_base_global     = np.array([0,0,0], dtype=np.float64)     # linear velocity of the base in the global coord system
        self.angular_base        = np.array([0,0,0], dtype=np.float64)     # angular velocity of the base
        self.angular_base_global = np.array([0,0,0], dtype=np.float64)     # angular velocity of the base in the global coord system
        self.roll:float      = 0.0
        self.pitch:float     = 0.0
        self.yaw:float       = 0.0
        self.base_freq:float = 0.0
        # UPDATES TARGETS FOOTS: 
        # Right targets foots
        self.t_2_r_rf = None 
        self.t_1_r_rf = None
        self.r_rf     = None
        self.t_2_r_rm = None
        self.t_1_r_rm = None
        self.r_rm     = None
        self.t_2_r_rr = None
        self.t_1_r_rr = None
        self.r_rr     = None
        # Left targets foots
        self.t_2_r_lf = None
        self.t_1_r_lf = None
        self.r_lf     = None
        self.t_2_r_lm = None
        self.t_1_r_lm = None
        self.r_lm     = None
        self.t_2_r_lr = None
        self.t_1_r_lr = None
        self.r_lr     = None

        self.joint_positions_rf = None
        self.joint_positions_rm = None
        self.joint_positions_rr = None
        self.joint_positions_lf = None
        self.joint_positions_lm = None
        self.joint_positions_lr = None

        self.joint_velocities_rf = [0,0,0]
        self.joint_velocities_rm = [0,0,0]
        self.joint_velocities_rr = [0,0,0]
        self.joint_velocities_lf = [0,0,0]
        self.joint_velocities_lm = [0,0,0]
        self.joint_velocities_lr = [0,0,0]
        self.joint_velocities_rf_t1 = [0,0,0]
        self.joint_velocities_rm_t1 = [0,0,0]
        self.joint_velocities_rr_t1 = [0,0,0]
        self.joint_velocities_lf_t1 = [0,0,0]
        self.joint_velocities_lm_t1 = [0,0,0]
        self.joint_velocities_lr_t1 = [0,0,0]
        self.joint_velocities_rf_t2 = [0,0,0]
        self.joint_velocities_rm_t2 = [0,0,0]
        self.joint_velocities_rr_t2 = [0,0,0]
        self.joint_velocities_lf_t2 = [0,0,0]
        self.joint_velocities_lm_t2 = [0,0,0]
        self.joint_velocities_lr_t2 = [0,0,0]

        self.foot_contact_state_rf:int = 0
        self.foot_contact_state_rm:int = 0
        self.foot_contact_state_rr:int = 0
        self.foot_contact_state_lf:int = 0
        self.foot_contact_state_lm:int = 0
        self.foot_contact_state_lr:int = 0

        self.thigh_contact_state_rf:int = 0
        self.thigh_contact_state_rm:int = 0
        self.thigh_contact_state_rr:int = 0
        self.thigh_contact_state_lf:int = 0
        self.thigh_contact_state_lm:int = 0
        self.thigh_contact_state_lr:int = 0

        self.shank_contact_state_rf:int = 0
        self.shank_contact_state_rm:int = 0
        self.shank_contact_state_rr:int = 0
        self.shank_contact_state_lf:int = 0
        self.shank_contact_state_lm:int = 0
        self.shank_contact_state_lr:int = 0

        self.phase_rf:float = 0.0
        self.phase_rm:float = 0.0
        self.phase_rr:float = 0.0
        self.phase_lf:float = 0.0
        self.phase_lm:float = 0.0
        self.phase_lr:float = 0.0

        self.efforts_rf:list = None 
        self.efforts_rm:list = None 
        self.efforts_rr:list = None 
        self.efforts_lf:list = None 
        self.efforts_lm:list = None 
        self.efforts_lr:list = None

        self.heightmaps_rf:list = None 
        self.heightmaps_rm:list = None 
        self.heightmaps_rr:list = None 
        self.heightmaps_lf:list = None 
        self.heightmaps_lm:list = None 
        self.heightmaps_lr:list = None
        # friction coefficients:
        self.foot_ground_friction_coefficients  = [1,1,1,1,1,1]
        self.external_force_applied_to_the_base = [0,0,0]
        self.gravity_vector = np.array([0,0,-1], dtype=np.float64)
        ########################################################################################################################################
        queue_size = 1
        # delta r publishers 
        self.pub_delta_r_rf = rospy.Publisher('/phantomx/delta_r/right_front',  Float64MultiArray,  queue_size=queue_size)
        self.pub_delta_r_rm = rospy.Publisher('/phantomx/delta_r/right_middle', Float64MultiArray,  queue_size=queue_size)
        self.pub_delta_r_rr = rospy.Publisher('/phantomx/delta_r/right_rear',   Float64MultiArray,  queue_size=queue_size)
        self.pub_delta_r_lf = rospy.Publisher('/phantomx/delta_r/left_front',   Float64MultiArray,  queue_size=queue_size)
        self.pub_delta_r_lm = rospy.Publisher('/phantomx/delta_r/left_middle',  Float64MultiArray,  queue_size=queue_size)
        self.pub_delta_r_lr = rospy.Publisher('/phantomx/delta_r/left_rear',    Float64MultiArray,  queue_size=queue_size)
        # delta r publishers list
        self.delta_r_publishers = [self.pub_delta_r_rf, self.pub_delta_r_rm, self.pub_delta_r_rr, 
                                   self.pub_delta_r_lf, self.pub_delta_r_lm, self.pub_delta_r_lr]
        # frequency publishers 
        self.pub_f_rf = rospy.Publisher('/phantomx/frec/right_front', Float64,  queue_size=queue_size)
        self.pub_f_rm = rospy.Publisher('/phantomx/frec/right_middle',Float64,  queue_size=queue_size)
        self.pub_f_rr = rospy.Publisher('/phantomx/frec/right_rear',  Float64,  queue_size=queue_size)
        self.pub_f_lf = rospy.Publisher('/phantomx/frec/left_front',  Float64,  queue_size=queue_size)
        self.pub_f_lm = rospy.Publisher('/phantomx/frec/left_middle', Float64,  queue_size=queue_size)
        self.pub_f_lr = rospy.Publisher('/phantomx/frec/left_rear',   Float64,  queue_size=queue_size)
        # frequency publishers list
        self.freq_publishers = [self.pub_f_rf, self.pub_f_rm, self.pub_f_rr,
                                self.pub_f_lf, self.pub_f_lm, self.pub_f_lr]
        self.pub_linear_vel_command = rospy.Publisher('/phantomx/linear_vel_command',   Float64MultiArray,  queue_size=queue_size)
        self.pub_angular_vel_command = rospy.Publisher('/phantomx/angular_vel_command',   Float64,  queue_size=queue_size)
        ########################################################################################################################################
        
        # Initialize first command
        self.linear_velocity_command, self.angular_velocity_command = self.define_desired_direction()
        
        ########################################################################################################################################
        
        # for reset the simulation
        self.max_timesteps_per_episode = max_timesteps_per_episode
        self.current_step = 0
        self.current_trajectory = 0
        # normalize the action space: [-1, 1]
        lower_limit_frec = -1
        upper_limit_frec = +1
        lower_limit_delta_r = -1
        upper_limit_delta_r = +1
        self.reward_range      = (-float('inf'), float('inf'))
        ## normalize action space: 24
        self.action_space = gym.spaces.Box(low=np.array([lower_limit_frec]  * 6 + [lower_limit_delta_r] * 18, dtype=np.float64),  
                                           high=np.array([upper_limit_frec] * 6 + [upper_limit_delta_r] * 18, dtype=np.float64),
                                           shape=(24,),  
                                           dtype=np.float64)
        # total teacher state: 105 (privilage dimension) + 175 (obs state) = 280
        self.state_representation_length:int = 175  # obs state 175
        self.privilaged_information_length:int = 105 # privilaged info 105
        #
        down_values, upper_values = self.limits_observation_space()
        self.observation_space = gym.spaces.Box(low=down_values,
                                                high=upper_values,
                                                shape=(self.state_representation_length + self.privilaged_information_length,),  
                                                dtype=np.float64)
        # Loggin for tensorboard
        self.vpr:list                                     = []
        self.wz:list                                      = []
        self.wpr:list                                     = []
        self.heading_error: list                          = []
        self.buffer_weights_target_smoothness_reward:list = []
        self.buffer_weights_foot_clearance_reward:list    = []
        self.buffer_weights_stable_angles:list            = []
        self.buffer_weights_linear_velocity_reward:list   = []
        self.buffer_weights_body_collision_reward:list    = []
        self.buffer_weights_angular_velocity_reward:list  = []
        self.buffer_weights_base_motion_reward:list       = []
        self.buffer_target_smoothness_reward:list         = []
        self.buffer_torque_reward:list                    = []
        self.buffer_stable_walk_reward:list               = []
        self.buffer_foot_clearance_reward:list            = []
        self.buffer_stable_angles:list                    = []
        self.buffer_linear_velocity_reward:list           = []
        self.buffer_body_collision_reward:list            = []
        self.buffer_angular_velocity_reward:list          = []
        self.buffer_base_motion_reward:list               = []
        ##########################################################################################
        logger.remove() # add this for remove the default logger
        logger.add(sys.stderr, format="{time} {level} {message}", filter="my_module", level="INFO")
        logger.add("file_{time}.log", rotation="100 MB", level="INFO")
        logger.info("Python's Phantomx environment initialized")
        time.sleep(0.5)
        ################################ Terrains ##############################################
        # paths to the model.sdf file:
        self.current_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
        # navigate one level up in the directory hierarchy
        self.root_base = os.path.dirname(self.current_directory)
        self.root_file = self.root_base + '/models/terrain/model.sdf'
        self.models = 'model://terrain/terrains_generated/' 
        ################################ Particles #############################################
        self.N_evaluate:int                = 10
        self.N_particles:int               = 10
        self.N_traj:int                    = 6

        self.k:int                         = 0
        self.l:int                         = 0
        self.m:int                         = 0

        self.P_replay:float                = 0.05
        self.P_transition:float            = 0.8
        self.transvesbility_theshold:float = 0.1
        self.transversability:list         = []
        self.measurement_probability        = dict() # avegare transversability for each terrain type  
        # init random particles:
        self.roughness_boundaries = (0.0, 0.05)
        self.frequency_boundaries = (0.2, 1.0)
        self.amplitude_boundaries = (0.2, 3.0)
        # map (0.2, 3) to (0.1,0.2) 
        self.amplitude_heightmap_bounderies = (0.12, 0.2) 
        self.heightmap_map_function = lambda x: np.interp(x, self.amplitude_boundaries, self.amplitude_heightmap_bounderies)
        # initialize with random particles easy terrains:
        particles = [np.array([np.round(float(np.random.uniform(low=0.02, high=0.03, size=1)),3),         # roughness (m): [0.0, 0.05]
                               np.round(float(np.random.uniform(low=0.20, high=0.40, size=1)),3) + 1e-4,  # frequency (f): [0.2, 1]
                               np.round(float(np.random.uniform(low=0.20, high=0.80, size=1)),3)],        # amplitude (a): [0.2, 3.0]
                               dtype=np.float64) for i in range(2*self.N_particles)]
        self.CTs_terrain_particles = particles[0:self.N_particles]
        # init replay memory:
        self.replay_memory        = deque(particles, maxlen=200*self.N_particles)  
        ########################################################################################################
        logger.info(f"CTs_terrain_particles: {self.CTs_terrain_particles}")
        self.new_terrain_generated = None
        logger.info(f"Number of evaluations: {self.N_evaluate}")
        logger.info(f"Number of particles: {self.N_particles}")
        logger.info(f"Number of trajectories per particle: {self.N_traj}")
        logger.info(f"Probability of sampling from the replay memory: {self.P_replay}")
        logger.info(f"Probability of transition to a new particle: {self.P_transition}")
        logger.info(f"Transversability threshold: {self.transvesbility_theshold}")
        logger.info(f"Lenght of the replay memory: {len(self.replay_memory)}")
        assert(len(self.CTs_terrain_particles) == self.N_particles)
        ####################################### END INIT ######################################################

    def step(self, action):
        """ Execute one time step within the environment. 
        
        In order to stabilize the frequency of the control system, most of the environment, thought not all
        was implemented in C++. The corresponding code, that needs to be running before executing the python code,
        is called server.cpp.
        The C++ code receives the 24 float vector containing the action to be performed through the socket
        defined in the init method of this class, it unpauses the simulation, executes the action, and after a time
        pauses the simulation again, returning through the socket and incomplete version of the observation vector
        to which we should add:
        1) Desired direction
        2) Gravity vector calculated using the quaternion
        3) Base velocities in its coordinate system
        4) Rearrenge base frequency
        5) Foot-ground friction coefficients
        6) External force applied to the base

        observation vector from C++ (obs):
        q = obs[0:4]
        base velocity = obs[4:7]
        angular rate = obs[7:10]
        position/velocities of each joint = obs[10:46] (6*3*2 = 36)
        FTG phases = obs[46:58]
        FTG frequncies = obs[58:64]
        Joint pos error history = obs[64:100] (6*3*2 = 36)
        Joint velocity history = obs[100:136] (6*3*2 = 36)
        r history = obs[136:172]
        terrain normal = obs[172:190] (6*3 = 18)
        height scans = obs[190:244] (6*9 = 54)
        contact forces = obs[244:250]
        foot contact states = obs[250:256]
        thigh contact states = obs[256:262]
        shank contact states = obs[262:268]
        r = obs[268:286]
        base frequency = obs[286]
        feet phase = obs[287:293]
        tibia efforts = obs[293:299]
        thigh efforts = obs[299:305]
        coxa  efforts = obs[305:311]
        """

        action = list(action) + [0.0]
        try:
            # sent the action to the server:
            packed_data = struct.pack(f"<{len(action)}f", *action)
            sent = self.client_socket.send(packed_data)
        except Exception as e:
            logger.error(f"An error occurred: {e}")
            logger.info("Close the connection and trying to reconnect...")
            # retried to connect
            self.client_socket.close()
            time.sleep(0.1)
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Connect to the server
            server_address = ('localhost', 12345)  
            self.client_socket.connect(server_address)
            return  [0]*(self.state_representation_length + self.privilaged_information_length), 0, True, True, {}

        num_values = 311
        data = []
        try:
            receivedData = self.client_socket.recv(num_values*4) 
            data = struct.unpack('f'*num_values, receivedData)
        except Exception as e:
            logger.error(f"An error occurred: {e}")
            logger.info("Close the connection and trying to reconnect...")
            # retried to connect
            self.client_socket.close()
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Connect to the server
            server_address = ('localhost', 12345)
            self.client_socket.connect(server_address)
            time.sleep(0.1)
            return  [0]*(self.state_representation_length + self.privilaged_information_length), 0, True, True, {}

        # UPDATES TARGETS FOOTS, check index in the c++ code (used for the reward function): 
        # right front 
        self.t_2_r_rf = [data[136], data[137], data[138]]
        self.t_1_r_rf = [data[139], data[140], data[141]]
        self.r_rf     = [data[268], data[269], data[270]]
        # right middle
        self.t_2_r_rm = [data[142], data[143], data[144]]
        self.t_1_r_rm = [data[145], data[146], data[147]]
        self.r_rm     = [data[271], data[272], data[273]]
        # right rear
        self.t_2_r_rr = [data[148], data[149], data[150]]
        self.t_1_r_rr = [data[151], data[152], data[153]]
        self.r_rr     = [data[274], data[275], data[276]]
        # left front
        self.t_2_r_lf = [data[154], data[155], data[156]]
        self.t_1_r_lf = [data[157], data[158], data[159]]
        self.r_lf     = [data[277], data[278], data[279]]
        # left middle
        self.t_2_r_lm = [data[160], data[161], data[162]]
        self.t_1_r_lm = [data[163], data[164], data[165]]
        self.r_lm     = [data[280], data[281], data[282]]
        # left rear
        self.t_2_r_lr = [data[166], data[167], data[168]]
        self.t_1_r_lr = [data[169], data[170], data[171]]
        self.r_lr     = [data[283], data[284], data[285]]
        # joint positions
        self.joint_positions_rf = [data[34], data[22], data[10]]
        self.joint_positions_rm = [data[36], data[24], data[12]]
        self.joint_positions_rr = [data[38], data[26], data[14]]
        self.joint_positions_lf = [data[40], data[28], data[16]]
        self.joint_positions_lm = [data[42], data[30], data[18]]
        self.joint_positions_lr = [data[44], data[32], data[20]]
        # joint velocities
        self.joint_velocities_rf = [data[35], data[23], data[11]]
        self.joint_velocities_rm = [data[37], data[25], data[13]]
        self.joint_velocities_rr = [data[39], data[27], data[15]]
        self.joint_velocities_lf = [data[41], data[29], data[17]]
        self.joint_velocities_lm = [data[43], data[31], data[19]]
        self.joint_velocities_lr = [data[45], data[33], data[21]]
        # joint velocities t-1
        self.joint_velocities_rf_t1 = [data[125], data[113], data[101]]
        self.joint_velocities_rm_t1 = [data[127], data[115], data[103]]
        self.joint_velocities_rr_t1 = [data[129], data[117], data[105]]
        self.joint_velocities_lf_t1 = [data[131], data[119], data[107]]
        self.joint_velocities_lm_t1 = [data[133], data[121], data[109]]
        self.joint_velocities_lr_t1 = [data[135], data[123], data[111]]
        # joint velocities t-2
        self.joint_velocities_rf_t2 = [data[124], data[112], data[100]]
        self.joint_velocities_rm_t2 = [data[126], data[114], data[102]]
        self.joint_velocities_rr_t2 = [data[128], data[116], data[104]]
        self.joint_velocities_lf_t2 = [data[130], data[118], data[106]]
        self.joint_velocities_lm_t2 = [data[132], data[120], data[108]]
        self.joint_velocities_lr_t2 = [data[134], data[122], data[110]]
        # foot's contact state (used for the reward function)
        self.foot_contact_state_rf = int(data[250])
        self.foot_contact_state_rm = int(data[251])
        self.foot_contact_state_rr = int(data[252])
        self.foot_contact_state_lf = int(data[253])
        self.foot_contact_state_lm = int(data[254])
        self.foot_contact_state_lr = int(data[255])
        # thigh's contact state (used for the reward function)
        self.thigh_contact_state_rf = int(data[256])
        self.thigh_contact_state_rm = int(data[257])
        self.thigh_contact_state_rr = int(data[258])
        self.thigh_contact_state_lf = int(data[259])
        self.thigh_contact_state_lm = int(data[260])
        self.thigh_contact_state_lr = int(data[261])
        # shank's contact state (used for the reward function)
        self.shank_contact_state_rf = int(data[262])
        self.shank_contact_state_rm = int(data[263])
        self.shank_contact_state_rr = int(data[264])
        self.shank_contact_state_lf = int(data[265])
        self.shank_contact_state_lm = int(data[266])
        self.shank_contact_state_lr = int(data[267])
        # phase (used for the reward function)
        self.phase_rf = float(data[287])
        self.phase_rm = float(data[288])
        self.phase_rr = float(data[289])
        self.phase_lf = float(data[290])
        self.phase_lm = float(data[291])
        self.phase_lr = float(data[292])
        # Efforts (used for the reward function)
        self.efforts_rf = [float(data[293]), float(data[299]), float(data[305])]
        self.efforts_rm = [float(data[294]), float(data[300]), float(data[306])]
        self.efforts_rr = [float(data[295]), float(data[301]), float(data[307])]
        self.efforts_lf = [float(data[296]), float(data[302]), float(data[308])]
        self.efforts_lm = [float(data[297]), float(data[303]), float(data[309])]
        self.efforts_lr = [float(data[298]), float(data[304]), float(data[310])]
        # Heightmaps (used for the reward function)
        self.heightmaps_rf = list()
        self.heightmaps_rm = list()
        self.heightmaps_rr = list()
        self.heightmaps_lf = list()
        self.heightmaps_lm = list()
        self.heightmaps_lr = list()
        # heightmaps
        for i in range(9):
            self.heightmaps_rf.append(data[190+i])
            self.heightmaps_rm.append(data[199+i])
            self.heightmaps_rr.append(data[208+i])
            self.heightmaps_lf.append(data[217+i])
            self.heightmaps_lm.append(data[226+i])
            self.heightmaps_lr.append(data[235+i])                            
        # observation
        qx, qy, qz, qw  = data[0], data[1], data[2], data[3]
        # gravity vector in lab frame
        # Quaternion representing the rotation from s1 to s2
        transform_to_robot_frame = np.array([[1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                                             [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                                             [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]]).T
        # represent the direction of gravity in the robot's local frame
        self.gravity_vector = transform_to_robot_frame.dot(np.array([0, 0, -1]))
        self.vel_base = transform_to_robot_frame.dot(np.array(data[4:7]))
        # represent the angular velocity of the robot from lab projected in their frame
        self.angular_base = transform_to_robot_frame.dot(np.array(data[7:10]))
        self.base_freq = data[286]
        # roll pitch yaw
        self.roll, self.pitch, self.yaw = euler_from_quaternion([qx, qy, qz, qw]) 
        #
        obs = [self.linear_velocity_command[0],                                          # Desired direction dim 2, 0
               self.linear_velocity_command[1],                                          # 1
               self.angular_velocity_command,                                   # 2 Desired turning direction: dim 1
               self.gravity_vector[0],                                           # 3 gravity vector in robot frame or pitch roll yaw: dim 3
               self.gravity_vector[1],                                           # 4
               self.gravity_vector[2],                                           # 5
               self.angular_base[0], self.angular_base[1], self.angular_base[2], # 6 angular velocity of the base in the robot frame: dim 3
               self.vel_base[0], self.vel_base[1], self.vel_base[2]]             # 7 linear velocity of the base in the robot frame:  dim 3
        # Obs total dim 12
        assert(len(obs) == 12)
        # 12 + (268 - 10) + 1 + 6 + 3 = 280
        # TODO: add friction coeff and external force sensors!!
        self.foot_ground_friction_coefficients = [1]*6
        self.external_force_applied_to_the_base = [0] *3
        obs += list(data[10:64]) + [self.base_freq]+ list(data[64:268]) + self.foot_ground_friction_coefficients + self.external_force_applied_to_the_base
        
        reward = self.get_reward()
        self.current_step += 1
        terminated = self.termination(self.pitch, self.roll)                    
        truncated = self.current_step >= self.max_timesteps_per_episode
        info = {}
        if self.current_step == 1: 
            logger.info(f"Start trajectory {self.m}, Direction command: {str(self.linear_velocity_command)} for the particle CT_{self.l}: {self.CTs_terrain_particles[self.l]}")
        # Print reward statistics for each component
        if terminated or truncated:
            # Populate info with statistics
            info = { 
                
                "stats": {
                    # "mean_weights_foot_clearance_reward"    : np.mean(self.buffer_weights_foot_clearance_reward),
                    # "mean_weights_linear_velocity_reward"   : np.mean(self.buffer_weights_linear_velocity_reward),
                    # "mean_weights_base_motion_reward"       : np.mean(self.buffer_weights_base_motion_reward),
                    # "mean_weights_target_smoothness_reward" : np.mean(self.buffer_weights_target_smoothness_reward),
                    # "mean_weights_angular_velocity_reward"  : np.mean(self.buffer_weights_angular_velocity_reward),
                    # "mean_weights_body_collision_reward"    : np.mean(self.buffer_weights_body_collision_reward),
                    "mean_foot_clearance_reward"            : np.mean(self.buffer_foot_clearance_reward),
                    "mean_linear_velocity_reward"           : np.mean(self.buffer_linear_velocity_reward),
                    "mean_base_motion_reward"               : np.mean(self.buffer_base_motion_reward),
                    "mean_target_smoothness_reward"         : np.mean(self.buffer_target_smoothness_reward),
                    "mean_torque_reward"                    : np.mean(self.buffer_torque_reward),
                    "mean_stable_walk_reward"               : np.mean(self.buffer_stable_walk_reward),
                    "mean_angular_velocity_reward"          : np.mean(self.buffer_angular_velocity_reward),
                    "mean_body_collision_reward"            : np.mean(self.buffer_body_collision_reward),
                    # "min_vpr"                               : np.min(self.vpr),
                    "mean_vpr"                              : np.mean(self.vpr),
                    # "max_vpr"                               : np.max(self.vpr),
                    "mean_wz"                               : np.mean(self.wz),
                    "mean_wpr"                               : np.mean(self.wpr),
                    "min_heading_error"                     : np.min(self.heading_error),
                    "mean_heading_error"                    : np.mean(self.heading_error),
                    "max_heading_error"                     : np.max(self.heading_error)
                }
            }
            # calculate transversability for the terreain
            self.transversability.append(self.compute_transversability(self.vpr, terminated))
            logger.info(f"End trajectory: {self.m}, transversability for the particle CT_{self.l}: {self.CTs_terrain_particles[self.l]} was: {np.round(self.transversability[-1],3)}")
            self.m +=1 # increment the trajectory index

        #logger.info(f"obs_velocities      : {obs[0:13]}",                file=sys.stderr)
        #logger.info(f"obs_tibias          : {list(data[10:22])}",        file=sys.stderr)
        #logger.info(f"obs_thighs          : {list(data[22:33])}",        file=sys.stderr)
        #logger.info(f"obs_coxas           : {list(data[33:46])}",        file=sys.stderr)
        #logger.info(f"obs_phases          : {list(data[46:58])}",        file=sys.stderr) 
        #logger.info(f"obs_frequencies     : {list(data[58:64])}",        file=sys.stderr) 
        #logger.info(f"obs_tibas_error     : {list(data[64:76])}",        file=sys.stderr) 
        #logger.info(f"obs_thigh_error     : {list(data[76:88])}",        file=sys.stderr)
        #logger.info(f"obs_coxas_error     : {list(data[88:100])}",       file=sys.stderr)
        #logger.info(f"obs_tibas_vel       : {list(data[100:112])}",      file=sys.stderr) 
        #logger.info(f"obs_thigh_vel       : {list(data[112:124])}",      file=sys.stderr)
        #logger.info(f"obs_coxas_vel       : {list(data[124:128])}",      file=sys.stderr)
        #logger.info(f"leg_targets         : {list(data[136:172])}",      file=sys.stderr)
        #logger.info(f"terrain_normals     : {list(data[172:190])}",      file=sys.stderr) 
        #logger.info(f"heightmaps          : {list(data[190:244])}",      file=sys.stderr)
        #logger.info(f"contact_forces      : {list(data[244:250])}",      file=sys.stderr)
        #logger.info(f"foot_contact_state  : {list(data[250:256])}",      file=sys.stderr)
        #logger.info(f"thigh_contact_state : {list(data[256:262])}",      file=sys.stderr)
        #logger.info(f"shank_contact_state : {list(data[262:267])}",      file=sys.stderr)       
        #logger.info("end step", file=sys.stderr)
        assert(len(obs) == self.state_representation_length + self.privilaged_information_length)
        assert(self.current_step <= self.max_timesteps_per_episode)    
        return obs, reward, terminated, truncated, info

    def reset(self, **kwargs):

        """ Reset the environment """
        
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation_client()
        time.sleep(0.1)

        # if len(self.wz)>0 :
        #     # plt.plot(self.wz, label='wz')
        #     # plt.plot(self.wpr, '--', label='wpr')
        #     # plt.plot(self.buffer_angular_velocity_reward, label='angular vel reward')
        #     plt.plot(self.heading_error, label='heading error')
        #     plt.xlabel('Time')
        #     plt.ylabel('Values')
        #     plt.legend()
        #     plt.title('Comparison of wz and wpr for command '+str(self.angular_velocity_command)+', Trajectory '+str(self.current_trajectory))
        #     plt.show()
            

        # Reset dr_i and f_i values on the low level control
        zero = Float64()
        zero.data = 0
        ang_max = np.pi/36
        pitch = np.random.uniform(-ang_max, ang_max)
        roll  = np.random.uniform(-ang_max, ang_max)
        yaw0  = np.random.uniform(-ang_max, ang_max)
        ang_max = np.pi/4
        action =[0]*(self.action_space.shape[0])
        for leg_idx, leg in enumerate(LEGS):
            yaw   = COXA_ORIENTATIONS[leg] + yaw0
            ang_coxa  = np.random.uniform(-ang_max, ang_max)
            ang_femur = np.random.uniform(-ang_max, ang_max)
            ang_tibia = np.random.uniform(-ang_max, ang_max)
            dr = self.direct_kinematics(ang_coxa, ang_femur, ang_tibia, yaw, pitch, roll)
            # dr = self.direct_kinematics(0, 0, 0, 0, 0, 0)
            action[leg_idx * 3]     = dr.item((0,0))
            action[leg_idx * 3 + 1] = dr.item((1,0))
            action[leg_idx * 3 + 2] = dr.item((2,0))
            action[leg_idx + 18] = 0

        # Set joints to default pattern walking
        for i in range(5):
            try:
                action_to_send = action + [1.0 if(i==4) else 0.0]
                packed_data = struct.pack(f"<{len(action_to_send)}f", *action_to_send)
                sent = self.client_socket.send(packed_data)
            except Exception as e:
                # Log the entire stack trace
                logger.error(f"An error occurred: {e}, leght: {len(sent)}")
                logger.info("Close the connection and retrying to connect...")
                # retried to connect
                self.client_socket.close()
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Connect to the server
                server_address = ('localhost', 12345)
                self.client_socket.connect(server_address)
                time.sleep(0.1)
 
            num_values = 311
            receivedData = [] 
            try:
                receivedData = self.client_socket.recv(num_values*4) 
                data = struct.unpack('f'*num_values, receivedData)
            except Exception as e:
                # Log the entire stack trace
                logger.error(f"An error occurred: {e}, leght: {len(receivedData)}")
                logger.info("Close the connection and retrying to connect...")
                # retried to connect
                self.client_socket.close()
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Connect to the server
                server_address = ('localhost', 12345)
                self.client_socket.connect(server_address)
                time.sleep(0.1)

        if self.current_trajectory >= self.N_traj*self.N_particles or self.current_trajectory == 0:
            self.current_trajectory = 0
        self.linear_velocity_command, self.angular_velocity_command = self.define_desired_direction()
        print("Linear Velocity Command: ", self.linear_velocity_command, 
            "Angular Velocity Command: ", self.angular_velocity_command)
        self.current_trajectory += 1

        qx, qy, qz, qw  = data[0], data[1], data[2], data[3]
        # gravity vector in lab frame
        # Quaternion representing the rotation from s1 to s2
        transform_to_robot_frame = np.array([[1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                                             [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                                             [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]]).T
        # represent the direction of gravity in the robot's local frame
        self.gravity_vector = transform_to_robot_frame.dot(np.array([0, 0, -1]))
        self.vel_base = transform_to_robot_frame.dot(np.array(data[4:7]))
        # Base velocity in global frame:
        self.vel_base_global = np.array(data[4:7])
        # represent the angular velocity of the robot from lab projected in their frame
        self.angular_base = transform_to_robot_frame.dot(np.array(data[7:10]))
        self.angular_base_global = np.array(data[7:10])
        # roll pitch yaw
        self.roll, self.pitch, self.yaw = euler_from_quaternion([qx, qy, qz, qw]) 
        self.base_freq = data[286]
        obs = [self.linear_velocity_command[0],                                  # 0 Desired direction(dim 2)
               self.linear_velocity_command[1],                                  # 1 "       "
               self.angular_velocity_command,                                    # 2 Desired turning direction: dim 1
               self.gravity_vector[0],                                           # 3 gravity vector in robot frame or pitch roll yaw: dim 3
               self.gravity_vector[1],                                           # 4
               self.gravity_vector[2],                                           # 5
               self.angular_base[0], self.angular_base[1], self.angular_base[2], # 6 angular velocity of the base in the robot frame: dim 3
               self.vel_base[0], self.vel_base[1], self.vel_base[2]]             # 7 linear velocity of the base in the robot frame:  dim 3
        # Obs total dim 2 + 1 + 3 + 3 + 3 = 12
        assert(len(obs) == 12)
        self.foot_ground_friction_coefficients = [1]*6
        self.external_force_applied_to_the_base = [0] *3
        obs += list(data[10:64]) + [self.base_freq]+ list(data[64:268]) + self.foot_ground_friction_coefficients + self.external_force_applied_to_the_base
        assert(len(obs) == self.state_representation_length + self.privilaged_information_length)
        '''
        #####################################################################################################################################
        ################################################# Delete the terrain ################################################################
        #####################################################################################################################################   
        deleted = False
        while(not deleted and self.current_step > 0):
            rospy.wait_for_service('/gazebo/delete_model')
            try:
                delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                response     = delete_model(f'terrain_{self.terrain_id}')
                deleted      = response.success
                if deleted:
                    logger.info(f"Terrain asociated to the particle CT_{self.l} = {self.CTs_terrain_particles[self.l]} was deleted")
            except rospy.ServiceException as e:
                logger.warning("Service call failed: {}".format(e))
                logger.warning(f"Terrain was not deleted, trying again...") 
        ######################################################################################################################################
        ################################################# Generate a new terrain #############################################################
        ######################################################################################################################################   
        if self.m == self.N_traj:
            
            logger.info(f"Transversabilities for particle CT_{self.l} = {self.CTs_terrain_particles[self.l]}: {self.transversability}")
            donw_threshold = 0.5
            up_threshold   = 0.9
            
            for i, tr in enumerate(self.transversability):
                if donw_threshold <= tr <= up_threshold:
                   self.transversability[i] = 1
                else:
                    self.transversability[i] = 0 
            
            res = np.mean(np.array(self.transversability))
            if str(self.CTs_terrain_particles[self.l]) in self.measurement_probability:
                self.measurement_probability[str(self.CTs_terrain_particles[self.l])].append(res)
            else:
                self.measurement_probability[str(self.CTs_terrain_particles[self.l])] = [res]
            
            logger.info(f"Average probabilities for particle CT_{self.l} = {self.CTs_terrain_particles[self.l]}: {np.round(self.measurement_probability[str(self.CTs_terrain_particles[self.l])],3)}")
            self.transversability = []
            self.m = 0
            self.l += 1
            if self.l == self.N_particles: 
                self.l = 0
                self.k += 1
                logger.info(f"Evaluation step: {self.k}")   
                if self.k == self.N_evaluate:
                    # Sum the total probability
                    total_probability = sum(chain(*list(self.measurement_probability.values()))) / self.N_evaluate
                    weights = np.array([np.mean(self.measurement_probability[str(self.CTs_terrain_particles[i])]) / (total_probability + 1e-8) 
                                                                                                                             for i in range(self.N_particles)])
                    # Resample N_particle parameters
                    indexes = self.choose_particles(weights)
                    logger.info(f"Evaluation step: {self.k}. Weights {weights} Particles selected indexes:{indexes}")
                    # Update the replay memory
                    for i in indexes:
                        particle = self.CTs_terrain_particles[i]
                        if not any(np.array_equal(arr, particle) for arr in self.replay_memory): self.replay_memory.append(particle)
                    # Sample from the replay memory and propagate
                    self.CTs_terrain_particles = self.sample_with_probability_and_propagate(particles=self.CTs_terrain_particles,
                                                                                            replay_memory=self.replay_memory, 
                                                                                            sample_probability=self.P_replay,
                                                                                            transition_probability=self.P_transition)
                    logger.info(f"New CTs_terrain_particles sampled: {str(self.CTs_terrain_particles)}")
                    logger.info(f"New replay memory: {str(self.replay_memory)}")
                    self.measurement_probability = dict()
                    self.k = 0
        ######################################################################################################################################
        ######################################################################################################################################
        ######################################################################################################################################
        self.new_terrain_generated = octaves_perlin(roughness = self.CTs_terrain_particles[self.l][0],
                                                    frequency = self.CTs_terrain_particles[self.l][1],
                                                    amplitude = self.CTs_terrain_particles[self.l][2])

        amplitude_z = np.round(self.heightmap_map_function(self.CTs_terrain_particles[self.l][2]),3)   
        parser_model_sdf(self.root_file, self.models + self.new_terrain_generated, height_z=amplitude_z)
        model_xml = open(self.root_file, 'r')
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.terrain_id = uuid.uuid4()
        spawn_model_client(model_name=f'terrain_{self.terrain_id}',
                        model_xml=model_xml.read(),
                        robot_namespace='teacher_terrain_trainner',
                        initial_pose=Pose(),
                        reference_frame='world')
        model_xml.close()
        time.sleep(0.1)
        logger.info(f"Terrain generated: {self.new_terrain_generated} for the particle CT_{self.l}={self.CTs_terrain_particles[self.l]} and trajectory {self.m}") 
        '''
        self.heightmaps_rf = None
        self.heightmaps_rm = None
        self.heightmapa_rr = None
        self.heightmaps_lf = None
        self.heightmaps_lm = None
        self.heightmaps_rr = None
        self.current_step = 0

        # empty the buffers
        self.vpr:list                                     = []
        self.wz:list                                      = []
        self.wpr:list                                     = []
        self.heading_error: list                          = []
        self.buffer_weights_target_smoothness_reward:list = []
        self.buffer_weights_foot_clearance_reward:list    = []
        self.buffer_weights_stable_angles:list            = []
        self.buffer_weights_linear_velocity_reward:list   = []
        self.buffer_weights_body_collision_reward:list    = []
        self.buffer_weights_angular_velocity_reward:list  = []
        self.buffer_weights_base_motion_reward:list       = []
        self.buffer_target_smoothness_reward:list         = []
        self.buffer_torque_reward:list                    = []
        self.buffer_stable_walk_reward:list               = []
        self.buffer_foot_clearance_reward:list            = []
        self.buffer_stable_angles:list                    = []
        self.buffer_linear_velocity_reward:list           = []
        self.buffer_body_collision_reward:list            = []
        self.buffer_angular_velocity_reward:list          = []
        self.buffer_base_motion_reward:list               = []

        return obs, {} 

    def render(self):
        pass

    def limits_observation_space(self):
        ''' Define limits of each observation made to be able to normalize observation vector'''

        # harcoded values for the state space:
        down_values = ([-1] * 3 +                   # desired linear and angular velocity direction
                    [-10] * 3 +                   # gravity vector
                    [-10, -10, -6.287] +          # angular base velocity (last value correspond to appŕox 360 degrees per second)
                    [-3, -3, -10] +               # linear base velocity
                    [-2.6179939] * 18 +           # joint positions
                    [-5.6548668] * 18 +           # joint velocities
                    [-1] * 12 +                   # FTG phase (sin/cos)
                    [-2*np.pi*FREQ_SAMPLE] * 6 +  # FTG frequencies (derivatives)
                    [-10] +                       # base frequency
                    [2*(-2.6179939)] * (6*3*2) +  # joint position error history
                    [-5.6548668] * (6*3*2) +      # joint velocity history
                    [-24.879] * (3*6*2) +         # Foot target history
                    # Privilaged Information:
                    [-1]*(3*6) +                  # Terrain Normals
                    [-10]*(9*6) +                 # Height Scans
                    [-50]*6 +                     # Contact Forces
                    [-1]*(3*6) +                  # Contact States
                    [-1]*6 +                      # Foot-ground friction coefficients
                    [-50]*3                       # External forces applied to the base
                    )
        upper_values=[-a for a in down_values]

        return np.array(down_values, dtype=np.float64), np.array(upper_values, dtype=np.float64)

    def define_desired_direction(self):
        # x = np.random.uniform(-1, 1)
        # y = np.random.uniform(-1, 1)
        # norm = np.linalg.norm([x, y])
        # desired_direction = [x/norm, y/norm] # Desired direction of the base
        # angular_velocity_command = np.random.randint(-1,2) # Generate uniformly random sample from {-1, 0, 1}
        index = np.random.randint(0,len(COMMAND_LINEAL))
        desired_direction = COMMAND_LINEAL[index]
        if index == 0: # This avoids selecting 0 for both velocities
            angular_velocity_command = COMMAND_ANGULAR[np.random.randint(1,len(COMMAND_ANGULAR))]
        else:
            angular_velocity_command = COMMAND_ANGULAR[np.random.randint(0,len(COMMAND_ANGULAR))]
        
        # Para que solo entrene en una dirección:
        desired_direction = COMMAND_LINEAL[1]
        angular_velocity_command = COMMAND_ANGULAR[0]

        linear_vel = Float64MultiArray()
        angular_vel = Float64()
        linear_vel.data = desired_direction
        angular_vel.data = angular_velocity_command
        
        self.pub_linear_vel_command.publish(linear_vel)
        self.pub_angular_vel_command.publish(angular_vel)
        return desired_direction, angular_velocity_command
    
    def termination(self, pitch, roll):
        '''
        Episode termination criterion
        '''
        terminated_fall = abs(pitch) > 0.7 or abs(roll) > 0.7 # Episode termination condition
        terminated = terminated_fall
        
        # base_vel = np.array([self.base_linear_velocity[0], self.base_linear_velocity[1]], dtype=np.float64) 
        # command_vel = np.array([self.desired_direction[0], self.desired_direction[1]], dtype=np.float64)
        # heading_error = np.arccos(np.dot(base_vel[0:2], command_vel[0:2])/np.linalg.norm(base_vel[0:2]))
        # self.heading_history.append(heading_error)
        # base_vel_ang = self.base_angular_velocity[2]
        # command_vel_ang = self.angular_velocity_command
        # self.w_history.append(base_vel_ang)
        
        # L = 100
        # w_lim = 0.1 # Minimum velocity to be considered non zero
        # if len(self.heading_history) > L:
        #     terminated_vel = abs(np.mean(self.heading_history[-L:])) > np.pi/2
        #     if command_vel_ang == 0:
        #         terminated_vel_ang = abs(np.mean(self.w_history[-L:])) > w_lim
        #     elif command_vel_ang > 0:
        #         terminated_vel_ang = np.mean(self.w_history[-L:]) < - w_lim
        #     else:
        #         terminated_vel_ang = np.mean(self.w_history[-L:]) > w_lim
        #     terminated = terminated or terminated_vel or terminated_vel_ang
        return terminated

    def direct_kinematics(self, ang_coxa, ang_femur, ang_tibia, yaw, pitch, roll):
        u = LF * np.cos(ang_femur) + LT * np.cos(ang_femur + ang_tibia)
        z = LF * np.sin(ang_femur) + LT * np.sin(ang_femur + ang_tibia)
        y = (u + LC) * np.sin(ang_coxa)
        x = (u + LC) * np.cos(ang_coxa)

        cy = np.cos(yaw)
        sy = np.sin(yaw)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cr = np.cos(roll)
        sr = np.sin(roll)
        
        R = np.matrix([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],\
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],\
                       [-sp  , cp*sr         , cp*cr]])
        return np.dot(R, np.matrix([[x], [y], [z]]))# t is not necessary because it's always (0,0,0)

    def random_terrains(self):

        terrain_particles = [np.array([np.round(float(np.random.uniform(low=self.roughness_boundaries[0], high=self.roughness_boundaries[1], size=1)),3),               # roughness (m): [0.0, 0.05]
                                       np.round(float(np.random.uniform(low=self.self.frequency_boundaries[0], high=self.frequency_boundaries[1],  size=1)),3) + 1e-4,  # frequency (f): [0.2, 1]
                                       np.round(float(np.random.uniform(low=self.amplitude_boundaries[0], high=self.amplitude_boundaries[1],  size=1)),3)],             # amplitude (a): [0.2, 3.0]
                                       dtype=np.float64) for i in range(self.N_particles)]
        
        assert(len(self.CTs_terrain_particles) == self.N_particles)
        return terrain_particles  

    def choose_particles(self, weights):

        """ Choose the particles with the highest weights """

        N = len(weights)
        assert(N == self.N_particles)
        indexes = []

        if sum(weights) != 0.0:        
            cumulative_weights = np.cumsum(weights)
            normalized_weights = cumulative_weights
            # Generate random numbers and perform resampling
            random_nums = np.random.rand(N)
            indexes = np.searchsorted(normalized_weights, random_nums)

        return list(set(indexes))
    
    def sample_with_probability_and_propagate(self, particles, 
                                                    replay_memory, 
                                                    sample_probability, 
                                                    transition_probability):
        
        res = []
        for particle in particles:
            random_number = np.random.rand()
            if random_number < sample_probability:
                particle = np.random.choice(replay_memory, size=1)
            random_number = np.random.rand()
            if random_number < transition_probability:
                # loc: The mean (center) of the distribution. In this case, it's set to 0.0. The mean is the average value around which the random numbers will be centered.
                # scale: The standard deviation of the distribution. It controls the spread or width of the distribution. Here, it's set to 0.01, indicating a relatively narrow distribution.
                # size: The number of random samples to generate. In this case, it's set to 1, so the result will be a NumPy array with three random numbers drawn from the specified normal distribution.
                tmp = particle.copy()
                particle[0] = float(np.clip(particle[0] + np.random.normal(loc=0.0, scale=0.01, size=1), self.roughness_boundaries[0], self.roughness_boundaries[1]))
                particle[1] = float(np.clip(particle[1] + np.random.normal(loc=0.0, scale=0.1,  size=1), self.frequency_boundaries[0], self.frequency_boundaries[1] + 1e-4))
                particle[2] = float(np.clip(particle[2] + np.random.normal(loc=0.0, scale=0.1,  size=1), self.amplitude_boundaries[0], self.amplitude_boundaries[1]))
                logger.info(f"Propagate Particle {str(tmp)} --> {str(particle)}")
            res.append(particle) 

        return res
        
    def compute_transversability(self, ls_vpr:list, termination:bool):
    
        """ Compute the transversability for a given trajectory """
        # filter the list of velocities > 0.2, if velocities > 0..2 them 1 else 0
        ls_tmp = [1.0 if vpr > self.transvesbility_theshold else 0.0 for vpr in ls_vpr]
        ls_tmp[-1] =  0 if termination else ls_tmp[-1]
        return np.mean(ls_tmp)

    ############################################################## Reward #########################################################################################
    def get_reward(self):
        return self.reward_ETH()
        # return self.reward_simple()

    def reward_simple(self):
        weights = [1, 0.05, 0.5, 1]
        
        # Calculate the acceleration of each joint and add it:
        a = 0
        for i in [0,1,2]:
            a -= abs(self.joint_velocities_rf[i] -2 *self.joint_velocities_rf_t1[i] +self.joint_velocities_rf_t2[i])
            a -= abs(self.joint_velocities_rm[i] -2 *self.joint_velocities_rm_t1[i] +self.joint_velocities_rm_t2[i])
            a -= abs(self.joint_velocities_rr[i] -2 *self.joint_velocities_rr_t1[i] +self.joint_velocities_rr_t2[i])
            a -= abs(self.joint_velocities_lf[i] -2 *self.joint_velocities_lf_t1[i] +self.joint_velocities_lf_t2[i])
            a -= abs(self.joint_velocities_lm[i] -2 *self.joint_velocities_lm_t1[i] +self.joint_velocities_lm_t2[i])
            a -= abs(self.joint_velocities_lr[i] -2 *self.joint_velocities_lr_t1[i] +self.joint_velocities_lr_t2[i])
        
        reward = weights[0] * self.vel_base_global[0] - weights[1] * a - weights[2] * abs(self.roll) - weights[3] * self.vel_base_global[1]
            

        self.buffer_linear_velocity_reward.append(0)
        self.buffer_angular_velocity_reward.append(0)
        self.buffer_base_motion_reward.append(0)
        self.buffer_foot_clearance_reward.append(0) 
        self.buffer_body_collision_reward.append(0)
        self.buffer_target_smoothness_reward.append(0)
        self.buffer_torque_reward.append(0)
        self.buffer_stable_walk_reward.append(0)
        
        dir_command_norm = np.linalg.norm(self.linear_velocity_command[0:2])
        dir_command_normalized = np.array([self.linear_velocity_command[0]/dir_command_norm, self.linear_velocity_command[1]/dir_command_norm], dtype=np.float64)\
                                                                        if dir_command_norm > 1e-2 else self.linear_velocity_command[0:2]
        v_pr = np.dot(self.vel_base_global[0:2], dir_command_normalized[0:2]) 
        self.vpr.append(v_pr)
        heading_error = math.atan2(self.vel_base_global[1], self.vel_base_global[0]) - math.atan2(dir_command_normalized[1], dir_command_normalized[0])
        self.heading_error.append(heading_error)
        return reward
    
    def reward_ETH(self):
        
        """ The reward function is a weigd sum of the following terms """

        weights = np.array([0.05,       # linear_velocity_reward    [0,1]
                            0.05,       # angular_velocity_reward   [0,1]
                            0.05,#0.04,       # base_motion_reward        [0,2]
                            0.08,#0.01,       # foot_clearance_reward     [0,1]
                            0.03,#0.02,       # body_collision_reward     [-12,0]  ?
                            0.005,#0.00025,      # target_smoothness_reward  [-inf,0]
                            0,#0.00002,    # torque_reward             [-inf,0]
                            0#0.01        # stable_walk_reward        [-??,1]
                            ], dtype=np.float64)
        
        functions = np.array([self.linear_velocity_reward(),
                              self.angular_velocity_reward(),
                              self.base_motion_reward(),
                              self.foot_clearance_reward(),
                              self.body_collision_reward(),
                              self.target_smoothness_reward(),
                              self.torque_reward(),
                              self.stable_walk_reward()], dtype=np.float64)
        
        reward = np.dot(weights, functions)
        self.buffer_weights_linear_velocity_reward.append(functions[0]*weights[0])
        self.buffer_weights_angular_velocity_reward.append(functions[1]*weights[1])
        self.buffer_weights_base_motion_reward.append(functions[2]*weights[2])
        self.buffer_weights_foot_clearance_reward.append(functions[3]*weights[3]) 
        self.buffer_weights_body_collision_reward.append(functions[4]*weights[4])
        self.buffer_weights_target_smoothness_reward.append(functions[5]*weights[5])
        self.buffer_linear_velocity_reward.append(functions[0])
        self.buffer_angular_velocity_reward.append(functions[1])
        self.buffer_base_motion_reward.append(functions[2])
        self.buffer_foot_clearance_reward.append(functions[3]) 
        self.buffer_body_collision_reward.append(functions[4])
        self.buffer_target_smoothness_reward.append(functions[5])
        self.buffer_torque_reward.append(functions[6])
        self.buffer_stable_walk_reward.append(functions[7])

        dir_command_norm = np.linalg.norm(self.linear_velocity_command[0:2])
        dir_command_normalized = np.array([self.linear_velocity_command[0]/dir_command_norm, self.linear_velocity_command[1]/dir_command_norm], dtype=np.float64)\
                                                                        if dir_command_norm > 1e-2 else self.linear_velocity_command[0:2]
        v_pr = np.dot(self.vel_base[0:2], dir_command_normalized[0:2]) 
        self.vpr.append(v_pr)
        heading_error = math.atan2(self.vel_base[1], self.vel_base[0]) - math.atan2(dir_command_normalized[1], dir_command_normalized[0])
        self.heading_error.append(heading_error)
        
        return reward

    def linear_velocity_reward(self):
        """
        Linear Velocity Reward (r_lv): This term maximizes v pr =( BIB v ) xy · ( BIB v̂ T ) xy , 
        which is the base linear velocity projected onto the command direction.
        """
        
        # dot product between the desired direction and the velocity of the base in the x-y plane
        dir_command_norm = np.linalg.norm(self.linear_velocity_command[0:2])
        dir_command_normalized = np.array([self.linear_velocity_command[0]/dir_command_norm, self.linear_velocity_command[1]/dir_command_norm], dtype=np.float64)\
                                                                        if dir_command_norm > 1e-2 else self.linear_velocity_command[0:2]
        # Dot product between the desired direction and the velocity of the base in the x-y plane
        # v_pr = ( BIB v ) xy · ( BIB v̂ T ) xy
        # v_pr = |vel_base| *  cos(theta)
        v_pr = np.dot(self.vel_base[0:2], dir_command_normalized[0:2]) 

        LINEAL_VEL_THRESHOLD = 0.1
        if v_pr < LINEAL_VEL_THRESHOLD:
            # res = np.exp(-2*(v_pr - LINEAL_VEL_THRESHOLD)**2)
            res = np.exp(-200*(v_pr - LINEAL_VEL_THRESHOLD)**2)
        elif v_pr >= LINEAL_VEL_THRESHOLD:
            res = 1.0
        # if v_pr < LINEAL_VEL_THRESHOLD:
        #     res = np.exp(-2000.0*(v_pr - LINEAL_VEL_THRESHOLD)**2)
        # elif v_pr >= LINEAL_VEL_THRESHOLD:
        #     res = 1.0

        # # norm of vel_comander
        # norm = np.linalg.norm(dir_command_normalized[0:2])
        # tolerance = 1e-2
        # if norm < tolerance: res = 0.0 #-np.linalg.norm(self.vel_base[0:2]) # 
        return res

    def base_motion_reward(self):

        """  Base Motion Reward (r_b): This term penalizes the velocity or-
        thogonal to the target direction and the roll and pitch rates such
        that the base is stable during the locomotion. """

        dir_command_norm = np.linalg.norm(self.linear_velocity_command[0:2])
        # dot product between the desired direction and the velocity of the base in the x-y plane
        # v_pr = ( BIB v ) xy · ( BIB v̂ T ) xy
        v_pr = np.dot(self.vel_base[0:2], self.linear_velocity_command[0:2])

        if dir_command_norm < 1e-2: v_pr = 0.0

        v0                = np.linalg.norm(self.vel_base[0:2] - v_pr * self.linear_velocity_command[0:2])
        base_angular_norm = np.linalg.norm(self.angular_base[0:2]) # w_x  w_y 

        # res =  np.exp(-1.5*(v0**2)) + np.exp(-1.5*(base_angular_norm**2)) 
        # res =  np.exp(-50*(v0**2)) + np.exp(-50*(base_angular_norm**2)) 
        res =  np.exp(-1000.5*(v0**2)) + np.exp(-1000.5*(base_angular_norm**2)) 
        return res
    
    def target_smoothness_reward(self):

        """ Smoothness Reward (r_s): The magnitude of the second order
            finite difference derivatives of the target foot positions are penalized
            such that the generated foot trajectories become smoother. """

        rs = 0
        rs -= np.linalg.norm(np.array(self.r_rf) - 2* np.array(self.t_1_r_rf) + np.array(self.t_2_r_rf))
        rs -= np.linalg.norm(np.array(self.r_rm) - 2* np.array(self.t_1_r_rm) + np.array(self.t_2_r_rm))
        rs -= np.linalg.norm(np.array(self.r_rr) - 2* np.array(self.t_1_r_rr) + np.array(self.t_2_r_rr))
        rs -= np.linalg.norm(np.array(self.r_lf) - 2* np.array(self.t_1_r_lf) + np.array(self.t_2_r_lf))
        rs -= np.linalg.norm(np.array(self.r_lm) - 2* np.array(self.t_1_r_lm) + np.array(self.t_2_r_lm))
        rs -= np.linalg.norm(np.array(self.r_lr) - 2* np.array(self.t_1_r_lr) + np.array(self.t_2_r_lr))
        return rs
        # r_vec_0 = np.array(self.r_rf + self.r_rm + self.r_rr + self.r_lr + self.r_lm + self.r_lf, dtype=np.float64)
        # r_vec_1 = np.array(self.t_1_r_rf + self.t_1_r_rm + self.t_1_r_rr + self.t_1_r_lr + self.t_1_r_lm + self.t_1_r_lf, dtype=np.float64)
        # r_vec_2 = np.array(self.t_2_r_rf + self.t_2_r_rm + self.t_2_r_rr + self.t_2_r_lr + self.t_2_r_lm + self.t_2_r_lf, dtype=np.float64)
        
        # return -np.linalg.norm(r_vec_0 - 2*r_vec_1 + r_vec_2)

    def angular_velocity_reward(self):
        
        """  Angular Velocity Reward (r_av): We motivate the agent to turn as
        fast as possible along the base z-axis when ( BIB ω̂ T ) z is nonzero. """

        w_z = self.angular_base[2]   #
        self.wz.append(w_z)
        w_pr = w_z*self.angular_velocity_command  # w_z and self.angular_velocity_command are both scalar
        self.wpr.append(w_pr)
        ANGULAR_VEL_THRESHOLD = 0.1
        # res =  np.exp(-1.5*(w_pr - ANGULAR_VEL_THRESHOLD)**2) if w_pr < ANGULAR_VEL_THRESHOLD else 1.0
        res =  np.exp(-200*(w_pr - ANGULAR_VEL_THRESHOLD)**2) if w_pr < ANGULAR_VEL_THRESHOLD else 1.0
        # if abs(self.angular_velocity_command) < 1e-2: res = 0.0 # np.exp(-1000.5*(w_z**2))
        if abs(self.angular_velocity_command) < 1e-2: 
            res = np.exp(-1000.5*(w_z**2))
        # else:
        #     res =  np.exp(-1000.5*(w_pr - ANGULAR_VEL_THRESHOLD)**2) if w_pr < ANGULAR_VEL_THRESHOLD else 1.0
        return res
    
    def body_collision_reward(self):
        # TODO: body collisions are missing
        """
        Body Collision Reward (rbc): We want to penalize undesirable
        contact between the robot's body parts and the terrain to avoid
        hardware damage
        """

        return -sum([self.thigh_contact_state_rf, self.thigh_contact_state_rm, self.thigh_contact_state_rr,   
                     self.thigh_contact_state_lf, self.thigh_contact_state_lm, self.thigh_contact_state_lr,
                     self.shank_contact_state_rf, self.shank_contact_state_rm, self.shank_contact_state_rr,   
                     self.shank_contact_state_lf, self.shank_contact_state_lm, self.shank_contact_state_lr])

    def foot_clearance_reward(self):

        """ Foot Clearance Reward (r_{f_c}): When a leg is in swing phase, i.e.,
            φ i ∈ [ π, 2π ) , the robot should lift the corresponding foot higher
            than the surroundings to avoid collision. """
        
        swing = lambda phase: 1.0 if np.pi <= phase < 2*np.pi else 0.0
        swing_phase = lambda phase, heightmap, z_leg: 1 if np.pi <= phase < 2*np.pi and max(heightmap) <= z_leg else 0

        swing_total = sum([swing(self.phase_rf),
                           swing(self.phase_rm),
                           swing(self.phase_rr),
                           swing(self.phase_lf),
                           swing(self.phase_lm),
                           swing(self.phase_lr)])
        if swing_total == 0: return 0
        # print(sum([swing_phase(self.phase_rf, self.heightmaps_rf, self.r_rf[2]),
        #             swing_phase(self.phase_rm, self.heightmaps_rm, self.r_rm[2]),
        #             swing_phase(self.phase_rr, self.heightmaps_rr, self.r_rr[2]),
        #             swing_phase(self.phase_lf, self.heightmaps_lf, self.r_lf[2]),
        #             swing_phase(self.phase_lm, self.heightmaps_lm, self.r_lm[2]),
        #             swing_phase(self.phase_lr, self.heightmaps_lr, self.r_lr[2])])/swing_total)
        return sum([swing_phase(self.phase_rf, self.heightmaps_rf, self.r_rf[2]),
                    swing_phase(self.phase_rm, self.heightmaps_rm, self.r_rm[2]),
                    swing_phase(self.phase_rr, self.heightmaps_rr, self.r_rr[2]),
                    swing_phase(self.phase_lf, self.heightmaps_lf, self.r_lf[2]),
                    swing_phase(self.phase_lm, self.heightmaps_lm, self.r_lm[2]),
                    swing_phase(self.phase_lr, self.heightmaps_lr, self.r_lr[2])])/swing_total
        # return sum([swing_phase(self.phase_rf, self.heightmaps_rf, self.r_rf[2]),
        #             swing_phase(self.phase_rm, self.heightmaps_rm, self.r_rm[2]),
        #             swing_phase(self.phase_rr, self.heightmaps_rr, self.r_rr[2]),
        #             swing_phase(self.phase_lf, self.heightmaps_lf, self.r_lf[2]),
        #             swing_phase(self.phase_lm, self.heightmaps_lm, self.r_lm[2]),
        #             swing_phase(self.phase_lr, self.heightmaps_lr, self.r_lr[2])])
    
    def torque_reward(self):
        # TODO: still not implemented
        """
        Torque Reward (r tau ): We penalize joint torques to prevent
        damaging the joint actuators during deployment and to reduce energy consumption
        """
        rs = 0
        rs += sum(abs(x) for x in self.efforts_rf)
        rs += sum(abs(x) for x in self.efforts_rm)
        rs += sum(abs(x) for x in self.efforts_rr)
        rs += sum(abs(x) for x in self.efforts_lf)
        rs += sum(abs(x) for x in self.efforts_lm)
        rs += sum(abs(x) for x in self.efforts_lr)
        # print(self.efforts_rf, self.efforts_rm, self.efforts_rr, self.efforts_lf, self.efforts_lm, self.efforts_lr)
        # print(rs)
        return rs
    
    def stable_walk_reward(self):
        '''
        This term rewards the agent for keeping 3 or more legs on the floor, so that the walk is more stable.
        '''
        # stand_legs = sum([self.foot_contact_state_rf, self.foot_contact_state_rm, self.foot_contact_state_rr,   
        #                   self.foot_contact_state_lf, self.foot_contact_state_lm, self.foot_contact_state_lr])
        # reward = 1 if stand_legs >= 3 else 0


        # stability_margin = self.static_stability_margin()
        # desired_value = 25
        # # desired_at_limit = -100
        # b = np.exp(-20*(desired_value)**2) # To make the function continuous
        # reward =  np.exp(-20*(stability_margin - desired_value)**2) if stability_margin > 0 else stability_margin + b
        # # reward = -(desired_at_limit+1)*(stability_margin - desired_value)**2/desired_value**2 + 1

        stability_margin = self.nesm() # for the phantomx this value is >=0 and <= 31.493
        reward = stability_margin/31.493

        return reward

    def static_stability_margin(self):
        ''' Calculates distance of the CoM to the support polygon. 
        The returned value cannot be greater than LC+LT+LF+y2
        	  Right: 				  Left:						
        				  |----| y1 = 10.34cm					
        Rear: (2)      \_____/        (3)          				
        	   		   |     |										
        	   		  /   z   \ 									
              (1)  -=----0----=---> y (4)    	+				
         	   		  \   |   /					| x1 = 12.48cm	
         	   		   |__|__|					+				
        Front:(0)      /  |  \        (5)   	    			
                 	      |										
                         V x 									
        				  |--| y2 = 6.164cm				

        LC = 5.053, LF = 6.828, LT = 12.998 and y2 = 6.164, so the returned value is <= 31.493.
        We define, for simpliciy, that if no legs are supported the returned value is the maximum
        possible when at least one is standing, which is -31.493.
        '''
        support = []
        if self.foot_contact_state_rf == 1: support += [self.determine_foot_position(self.joint_positions_rf,'rf')[0:2]]
        if self.foot_contact_state_rm == 1: support += [self.determine_foot_position(self.joint_positions_rm,'rm')[0:2]]
        if self.foot_contact_state_rr == 1: support += [self.determine_foot_position(self.joint_positions_rr,'rr')[0:2]]
        if self.foot_contact_state_lr == 1: support += [self.determine_foot_position(self.joint_positions_lr,'lr')[0:2]]
        if self.foot_contact_state_lm == 1: support += [self.determine_foot_position(self.joint_positions_lm,'lm')[0:2]]
        if self.foot_contact_state_lf == 1: support += [self.determine_foot_position(self.joint_positions_lf,'lf')[0:2]]

        if len(support) == 0: return -31.493
        if len(support) == 1: return -np.linalg.norm(np.array(support[0]))
        if len(support) == 2: return -self.distance_to_segment((0,0), support[0], support[1])

        # The support polygon is the convex hull of the points where the robot is standing
        convex_hull = self.find_convex_hull(support)
        convex_hull += [convex_hull[0]] # Close the support polygon

        # The base position is (0,0) because the legs are given wrt to the base
        base_position = (0, 0)

        # Calculate the distance from the point to the convex hull
        distance_to_hull = self.distance_to_convex_hull(base_position, convex_hull)
        if not(self.is_point_inside_polygon(base_position, convex_hull)):
            distance_to_hull = -1 * distance_to_hull
        
        return distance_to_hull
    
    def determine_foot_position(self, joint_pos, leg):
        ''' Calculates foot position wrt the base given it's 3 angles'''
        r = self.direct_kinematics(joint_pos[0], joint_pos[1], joint_pos[2], self.yaw + COXA_ORIENTATIONS[leg], self.pitch, self.roll)
        return [r.item((0,0)), r.item((1,0)), r.item((2,0))]

    def is_point_inside_polygon(self, point, vertices):
        point = Point(point[0], point[1])
        # Create a Shapely Polygon from the given vertices
        polygon = Polygon(vertices)
        # Check if the origin is inside the polygon
        return point.within(polygon)
        
    def find_convex_hull(self, vertices):
        # Create a ConvexHull object from the given vertices
        print(vertices)
        hull = ConvexHull(vertices)

        # Extract the vertices of the convex hull using the indices
        convex_hull_vertices = [vertices[i] for i in hull.vertices]

        return convex_hull_vertices

    def distance_to_segment(self, x, a, b):
        ''' Calculates the distance from 2D point x to segment ab'''
        a = np.array(a)
        b = np.array(b)
        x = np.array(x)
        aux = np.inner(b-a,x-a)/np.inner(b-a,b-a)
        if aux < 0:
            return np.linalg.norm(a-x)
        if aux > 1:
            return np.linalg.norm(b-x)
        
        p = a + aux * (b-a)
        return np.linalg.norm(p-x)

    def distance_to_convex_hull(self, point, convex_hull_vertices):
        # Calculate the minimum distance from a point to the convex hull
        min_distance = float('inf')

        for i in range(len(convex_hull_vertices) - 1):
            segment_start = convex_hull_vertices[i]
            segment_end = convex_hull_vertices[i + 1]
            dist = self.distance_to_segment(point, segment_start, segment_end)
            min_distance = min(min_distance, dist)

        return min_distance

    def nesm(self):
        '''
        Returns the Normalized Energy Stability Margin (NSESM), defined as:
        S_{NESM} = S_{ESM}/mg = min(h_i)
        where h_i is the variation of COG height during the tumble around the support polygon 
        i-th side.
        '''
        support = []
        if self.foot_contact_state_rf == 1: support += [self.determine_foot_position(self.joint_positions_rf,'rf')]
        if self.foot_contact_state_rm == 1: support += [self.determine_foot_position(self.joint_positions_rm,'rm')]
        if self.foot_contact_state_rr == 1: support += [self.determine_foot_position(self.joint_positions_rr,'rr')]
        if self.foot_contact_state_lr == 1: support += [self.determine_foot_position(self.joint_positions_lr,'lr')]
        if self.foot_contact_state_lm == 1: support += [self.determine_foot_position(self.joint_positions_lm,'lm')]
        if self.foot_contact_state_lf == 1: support += [self.determine_foot_position(self.joint_positions_lf,'lf')]

        if len(support) == 0: return 0
        if len(support) == 1: return 0
        if len(support) == 2: return self.h_i((0,0,0), support[0], support[1])

        # TODO: we're in 3D, so this "support polygon" isn't necessarily a polygon, and I can't find the convex
        # hull either. Think about this.
        support += [support[0]] # Close the support polygon

        # The base position is (0,0,0) because the legs are given wrt to the base
        base_position = (0, 0, 0)

        # Calculate the distance from the point to the convex hull
        min_hi = float('inf')

        for i in range(len(support) - 1):
            segment_start = support[i]
            segment_end = support[i + 1]
            hi = self.h_i(base_position, segment_start, segment_end)
            min_hi = min(min_hi, hi)
        
        return min_hi

    def h_i(self, x, a, b):
        '''Calculates h_i, the variation of COG height during the tumble around the support polygon 
        side defined by vertices a and b, given COG current position x.
        
        h_i = |R_i|(1-cos(theta))cos(psi)

        where Ri is the vector from the COG to the rotation axis, theta is the angle that Ri forms 
        with the vertical plane, and psi is the inclination angle of the rotation axis relative to 
        the horizontal plane.

        Knowing that both theta and psi are between 0 and pi/2, we can see that h_i can go from 0
        to (potentially) the maximum possible value for R_i.
        R_i cannot be greater than LC+LT+LF+y2
        	  Right: 				  Left:						
        				  |----| y1 = 10.34cm					
        Rear: (2)      \_____/        (3)          				
        	   		   |     |										
        	   		  /   z   \ 									
              (1)  -=----0----=---> y (4)    	+				
         	   		  \   |   /					| x1 = 12.48cm	
         	   		   |__|__|					+				
        Front:(0)      /  |  \        (5)   	    			
                 	      |										
                         V x 									
        				  |--| y2 = 6.164cm				

        LC = 5.053, LF = 6.828, LT = 12.998 and y2 = 6.164, so R_i <= 31.493.
        '''
        # We find the point P such that PX
        a = np.array(a)
        b = np.array(b)
        x = np.array(x)
        aux = np.inner(b-a,x-a)/np.inner(b-a,b-a)
        p = a + aux * (b-a)

        v = x - p

        n_vertical = np.cross(np.array([0,0,1]), b-a)
        sin_theta = abs(np.inner(v,n_vertical))/(np.linalg.norm(v)*np.linalg.norm(n_vertical))
        cos_theta = np.cos(np.arcsin(sin_theta))

        n_horizontal = np.array([0,0,1])
        sin_psi = abs(np.inner(v,n_horizontal))/(np.linalg.norm(v)*np.linalg.norm(n_horizontal))
        cos_psi = np.cos(np.arcsin(sin_psi))

        h_i = np.linalg.norm(v) * (1 - cos_theta) * cos_psi
        return h_i

# end class Phantomx
#-----------------------------------------#