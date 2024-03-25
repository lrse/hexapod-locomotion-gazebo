#!/usr/bin/env python3

# This script is used to train the teacher using the automatic terrain curriculum
import os 
import sys
import time
import rospy
import random
import subprocess
import numpy as np
from signal import SIGKILL
from std_srvs.srv import Empty
from std_msgs.msg import String, Float64, Float32MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from model_sdf import parser_model_sdf
from gazebo_msgs.srv import SetModelState, DeleteModel
from terrain_generator import hills_generator, stairs_generator

while True:
    pass


if __name__ == '__main__':

    while True:
        pass


    current_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
    # navigate one level up in the directory hierarchy
    root_base = os.path.dirname(current_directory)
    root_file = root_base + '/models/terrain/model.sdf'
    models = 'model://terrain/terrains_generated/' 

    N_evaluate = 10
    N_particle = 10
    N_traj     = 6

    # Algorithm S1. Teacher training with automatic terrain curriculum:

    # 1 - Initialize a replay memory, Sample N particle C_{T,0} s uniformly from C (Table S2). i, j = 0.

    #  Terrain  | grid size | frition coeficient | parameters (C_T) | range     |
    #  ---------------------------------------------------------------------------
    #  Hills    | 0.2 m     | N(0.7, 0.2)        | roughness (m)    | [0.0, 0.05]
    #           |           |                    | frequency        | [0.2, 1.0]
    #           |           |                    | amplitude (m)    | [0.2, 3.0]

    roughness_values = list(np.random.uniform(low=0.0, high=0.05, size=N_particle))
    frequency_values = list(np.random.uniform(low=0.2, high=1.0,  size=N_particle)) 
    amplitude_values = list(np.random.uniform(low=0.2, high=3.0,  size=N_particle))

    #rospy.wait_for_service('/gazebo/delete_model')
    #delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    #delete_model('terrain') 
    time.sleep(.1)
    default_terrain = hills_generator(0.01, 0.04, 1.9)
    parser_model_sdf(root_file, models + default_terrain)
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name='terrain_1',
                    model_xml=open(root_file, 'r').read(),
                    robot_namespace='teacher_terrain_trainner',
                    initial_pose=Pose(),
                    reference_frame='world')

    while True:
        pass    

    time.sleep(1.0)
    
    rospy.wait_for_service('/gazebo/delete_model')
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model('terrain') 

    time.sleep(1.0)
    
    for l in range(N_particle):
        # 6: Generate terrain using C^(l)_{T,j} 
        roughness, frequency, amplitude = roughness_values[l], frequency_values[l], amplitude_values[l]
        new_terrain_generated = hills_generator(roughness, frequency, amplitude)
        parser_model_sdf(root_file, models + new_terrain_generated)

        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name='terrain',
                        model_xml=open(root_file, 'r').read(),
                        robot_namespace='teacher_terrain_trainner',
                        initial_pose=Pose(),
                        reference_frame='world')

        for m in range(N_traj):
            rospy.wait_for_service('/gazebo/set_model_state')
            setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            modelState=ModelState()
            modelState.model_name= 'phantomx_gazebo'
            # Initialize robot at random position
            modelState.pose.position.x    = random.uniform(-15, 15)
            modelState.pose.position.y    = random.uniform(-15, 15)
            modelState.pose.position.z    = 0.6
            modelState.pose.orientation.x = 0
            modelState.pose.orientation.y = 0
            modelState.pose.orientation.z = random.uniform(-1, 1)
            modelState.pose.orientation.w = random.uniform(-1, 1)
            resp=setEstadoFun(modelState)

            time.sleep(5.0)
            # Run policy

        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model('terrain') 
        time.sleep(.1)