import time
import numpy as np
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv  import ApplyBodyWrenchRequest
from gazebo_msgs.srv  import ApplyBodyWrench
from std_srvs.srv import Empty
from env import Phantomx
from debugging_wrapper import DebugWrapper
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

# README:

# Lanzar el nodo ros pero comentando en Robots/phantomx/src/phantomx/launch/run_robot_gazebo.launch: 
# <include file="$(find phantomx_controller)/launch/robot_controller.launch"/> 
# porque es un nodo que escribe sobre los joints del robot y no queremos que se 
# manden mensajes cruzados, ojo con la red neuronal.

# Ejecutar python main_test.py para probar los joints del robot

def set_joint_angles(coxa = 0.0, 
                     thigh= 0.0, 
                     tibia= 0.0, 
                     side='right_front'):
    
    """  Define the angles of the leg joints of the hexapod robot.
            coxa  (float64): angle of the coxa joint
            thigh (float64): angle of the thigh joint
            tibia (float64): angle of the tibia joint
            side  (float64): side of the leg to be moved """
     
    legs_coordenates = {'right_front':  [0, 6, 12],
                        'right_middle': [1, 7, 13],
                        'right_rear':   [2, 8, 14],
                        'left_front':   [5, 11, 17],
                        'left_middle':  [4, 10, 16],
                        'left_rear':    [3, 9, 15]}

    assert side in legs_coordenates.keys(), "Invalid leg side"

    command_topics = [  # coxas 
                        "/phantomx/j_c1_rf_position_controller/command",		 #0
                        "/phantomx/j_c1_rm_position_controller/command",         #1
                        "/phantomx/j_c1_rr_position_controller/command",         #2
                        "/phantomx/j_c1_lr_position_controller/command",         #3
                        "/phantomx/j_c1_lm_position_controller/command",         #4
                        "/phantomx/j_c1_lf_position_controller/command",         #5
                        # thighs 
                        "/phantomx/j_thigh_rf_position_controller/command",      #6
                        "/phantomx/j_thigh_rm_position_controller/command",      #7
                        "/phantomx/j_thigh_rr_position_controller/command",      #8
                        "/phantomx/j_thigh_lr_position_controller/command",      #9
                        "/phantomx/j_thigh_lm_position_controller/command",      #10
                        "/phantomx/j_thigh_lf_position_controller/command",      #11
                        # tibas 
                        "/phantomx/j_tibia_rf_position_controller/command",      #12
                        "/phantomx/j_tibia_rm_position_controller/command",      #13
                        "/phantomx/j_tibia_rr_position_controller/command",      #14
                        "/phantomx/j_tibia_lr_position_controller/command",	     #15	  
                        "/phantomx/j_tibia_lm_position_controller/command",      #16
                        "/phantomx/j_tibia_lf_position_controller/command"]		 #17

    pubs = []
    for i in legs_coordenates[side]:
        pubs.append(rospy.Publisher(command_topics[i], Float64,queue_size=10))

    pubs[0].publish(coxa), pubs[1].publish(thigh), pubs[2].publish(tibia)
    # wait for the joints to move
    # rospy.sleep(0.1)



def apply_force_to_link(link_name='base_link', force=[0,0,0], duration=0):

        """ Apply a force to a link in the simulation.  
        
        Args:  
            link_name (str): The name of the link to apply the force to.
            force (list): The force to apply to the link.
            duration (float): The duration of the force.
        
        Returns:
            response (bool): Whether the force was applied or not.
        """

        rospy.wait_for_service('/gazebo/apply_body_wrench')
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        request = ApplyBodyWrenchRequest()
        request.body_name = link_name
        request.wrench.force.x = force[0]
        request.wrench.force.y = force[1]
        request.wrench.force.z = force[2]
        request.duration = rospy.Duration(duration)
        response = apply_wrench(request)

        return response.success


if __name__ == "__main__":

    # rospy.init_node("hexapod_testing")

    # Create the environment (this is only needed to use DebugWrapper)
    env = Phantomx(max_timesteps_per_episode=150)
    env = DebugWrapper(env)

    # unpause simulation service
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    LEG = ['right_front', 'right_middle', 'right_rear', 'left_front', 'left_middle', 'left_rear']
    action = [0] * 24

    while True:
        unpause_physics()
        for i in range(2):
            for i in range(6):
                set_joint_angles(coxa  = 0.0, thigh = 0.0, tibia = 0.0, side = LEG[i])
            env.step(action)
            unpause_physics()
        print("Sleep for 10 secs")
        # rospy.sleep(2.) # Wait for robot to stabilize
        obs = env.reset()
        unpause_physics()
        rospy.sleep(2.) # Wait for robot to stabilize
        print("Done resetting")
        for i in range(2,10000):
            angle = 10 * np.pi/180
            unpause_physics()
            set_joint_angles(coxa  = 0.0, 
                            thigh = angle, 
                            tibia = 0.0, 
                            side = LEG[0])
            
            # apply force to the base
            # apply_force_to_link('base_link', [100, 0, 0], 1)
            # rospy.sleep(0.1)

            obs, reward, terminated, truncated , info = env.step(action)
            if terminated or truncated:
                print("Episode finished after {} timesteps".format(i+1))
                break