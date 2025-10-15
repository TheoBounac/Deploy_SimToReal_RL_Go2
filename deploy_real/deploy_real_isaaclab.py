############ LIBRARIES #############
from typing import Union           #
import numpy as np                 #
import time                        #
import torch                       #
import sys                         #
import numpy as np                 #
import matplotlib.pyplot as plt    #
import pandas as pd                #
from math import *                 #
import pygame                      #
import rclpy                       #
from rclpy.node import Node        #
from geometry_msgs.msg import Twist#
####################################

######### To point the path to where unitree_sdk2_python is stored ########
sys.path.append('.')                                                      #
sys.path.append('..')                                                     #
sys.path.append('/home/theo/deploy/unitree_sdk2_python') # TO MODIFY !!!  #
###########################################################################


from unitree_sdk2_python.unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2_python.unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2_python.unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_

from unitree_sdk2_python.unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2_python.unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo

from unitree_sdk2_python.unitree_sdk2py.utils.crc import CRC
from unitree_sdk2_python.unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2_python.unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2_python.unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd_go
from common.rotation_helper import get_gravity_orientation, transform_imu_data
from common.remote_controller import RemoteController, KeyMap
from config import Config
import deploy_real.node_kalman as node_kalman


class Controller():
    def __init__(self, config: Config) -> None:
        # Config file for Go2
        self.config = config

        # Initialization of the controller
        self.remote_controller = RemoteController()

        # [Etape] 1. LOADING POLICY
        print("3] ---> CHARGEMENT DE LA POLITIQUE")
        self.policy = torch.jit.load("/home/theo/deploy/pre_train/go2/policy_rough_2.pt")

        # Initialization of variables for startup
        self.defaut_isaac = [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1, 1, -1.5, -1.5, -1.5, -1.5] # Default angles for Isaac’s convention (the one used by the policy)
        self.base_lin_vel = np.array([0, 0, 0])
        self.cmd = np.array([0.0, 0.0, 0.0])
        self.qj = np.zeros(config.num_actions, dtype=np.float32)
        self.dqj = np.zeros(config.num_actions, dtype=np.float32)
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.target_dof_pos = self.defaut_isaac.copy()
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
    
        # Data to raise the robot to the default position 
        self.dt = 0.002  
        self.startPos = [0.0] * 12
        self.duration_1 = 500
        self.duration_2 = 500
        self.duration_3 = 1000
        self.duration_4 = 900
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0
        self.firstRun = True
        self.counter = 0

        # Target positions to lift the Go2 to the default position
        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self._targetPos_2 = [-0.1,  0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1, -1.5, 0.1, 1, -1.5]
        self._targetPos_3 = self._targetPos_2

        # Data for the velocity estimator (the larger the window_size, the smoother the velocity calculation)
        window_size = 20
        self.vx_window = [0] * window_size
        self.vy_window = [0] * window_size
        self.vz_window = [0] * window_size

        # List for displaying the plots
        self.L_base_vel_cmd_input_1 = []
        self.L_base_vel_cmd_input_2 = []
        self.L_base_vel_cmd_input_3 = []

        self.L_base_lin_vel_input_1 = []
        self.L_base_lin_vel_input_2 = []
        self.L_base_lin_vel_input_3 = []

        self.L_base_lin_vel_kalman_input_1 = []
        self.L_base_lin_vel_kalman_input_2 = []
        self.L_base_lin_vel_kalman_input_3 = []
        self.L_base_lin_vel_kalman_input_4 = []

        self.L_base_ang_vel_input_1 = []
        self.L_base_ang_vel_input_2 = []
        self.L_base_ang_vel_input_3 = []



        ### ROS2 communication with Kalman filter which publishes on "/odometry/filtered" ###
        from rclpy.executors import MultiThreadedExecutor                                   #
        import threading                                                                    #
                                                                                            #
        rclpy.init()                                                                        #
        KOL = node_kalman.KalmanOdomListener()                                              #
                                                                                            #
        # MultiThreadedExecutor                                                             #
        executor = MultiThreadedExecutor()                                                  #
        executor.add_node(KOL)                                                              #
                                                                                            #
        # Lance l'exécuteur dans un thread                                                  #
        threading.Thread(target=executor.spin, daemon=True).start()                         #
        #####################################################################################

        # [Etape] 2. Initialization of the channels
        ##################################### Initialization of the channels ####################################
        # Initialization of the channels                                                                        #
        print("4] ----> INITIALISATION DES CHANNELS")                                                           #
                                                                                                                #
        self.lowcmd_publisher_ = ChannelPublisher(config.lowcmd_topic, LowCmdGo)                                #
        self.lowcmd_publisher_.Init()                                                                           #
                                                                                                                #
        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateGo)                         #
        self.lowstate_subscriber.Init(self.LowStateGoHandler, 10)                                               #
                                                                                                                #
        self.sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)                    #
        self.sportstate_subscriber.Init(self.SportStateMessageHandler, 10)                                      #
                                                                                                                #
        # Initialization of the messages for CMD and STATE                                                      #
        self.low_cmd = unitree_go_msg_dds__LowCmd_()                                                            #
        self.low_state = unitree_go_msg_dds__LowState_()                                                        #
                                                                                                                #
        # Wait for the subscriber to receive data                                                               #
        self.wait_for_low_state()                                                                               #
                                                                                                                #
        # Initialize the commands of the motors                                                                 #
        init_cmd_go(self.low_cmd, weak_motor=self.config.weak_motor)                                            #
        #########################################################################################################


    ########################## Init is called at the beginning to switch to Low Level #####################################
    def Init(self):                                                                                                       #
                                                                                                                          #
        self.sc = SportClient()                                                                                           #
        self.sc.SetTimeout(5.0)                                                                                           #
        self.sc.Init()                                                                                                    #
                                                                                                                          #
        self.msc = MotionSwitcherClient()                                                                                 #
        self.msc.SetTimeout(5.0)                                                                                          #
        self.msc.Init()                                                                                                   #
                                                                                                                          #
        status, result = self.msc.CheckMode()                                                                             #
        while result['name']:                                                                                             #
            self.sc.StandDown()        # Avant de commencer on met le robot en position allonge                           #
            self.msc.ReleaseMode()     # On libere le mode normal                                                         #
            print("3] ---> LE ROBOT EST EN POSITION ALLONGE ET LE MODE HAUT NIVEAU EST RELACHE -> PASSAGE EN BAS NIVEAU") #
            status, result = self.msc.CheckMode()                                                                         #
            time.sleep(1)                                                                                                 #
    #######################################################################################################################

        
    ######################## Fonction qui vérifie si le message low_state est bien recu ########################
    def wait_for_low_state(self):                                                                              #
        while self.low_state.tick == 0:                                                                        #
            time.sleep(self.config.control_dt)                                                                 #
        print("         Connecté au robot")                                                                    #
    ############################################################################################################


    ###### Handler functions are called as soon as the message in the parameter is received on a channel #######
    def LowStateGoHandler(self, msg: LowStateGo):                                                              #
        self.low_state = msg                                                                                   #
        self.remote_controller.set(self.low_state.wireless_remote)                                             #
        node_kalman.msg = msg                                                                                  #
                                                                                                               #
    def SportStateMessageHandler(self, sport_state_msg):                                                       #
        self.velocity = sport_state_msg.velocity  # Collecting the velocity                                    #                                                                                       
    ############################################################################################################


    ###################### Function that sends a command passed as a parameter to the robot ####################
    def send_cmd(self, cmd: LowCmdGo):                                                                         #
        cmd.crc = CRC().Crc(cmd)                                                                               #
        self.lowcmd_publisher_.Write(cmd)                                                                      #
    ############################################################################################################


    ######### Function that sends a null command while waiting for the start button to be pressed, which will trigger the robot lift ########
    def zero_torque_state(self):                                                                                                            #
        print("5] -----> LE ZERO TORQUE STATE EST EN COURS")                                                                                #
        print("                                                            ")                                                               #
        print("          ##################################################")                                                               #
        print("          # EN ATTENTE DU BOUTON START POUR LEVER LE ROBOT #")                                                               #
        print("          ##################################################")                                                               #
        print("                                                            ")                                                               #        
        while self.remote_controller.button[KeyMap.start] != 1:                                                                             #
            create_zero_cmd(self.low_cmd)                                                                                                   #
            self.send_cmd(self.low_cmd)                                                                                                     #
            time.sleep(self.config.control_dt)                                                                                              #
    #########################################################################################################################################


    # Function that raises the robot to a standing position and keeps it there until the "A" button is pressed, which will start the model #
    def move_to_default_pos(self):                                                                                                         
        print("6] ------> LE ROBOT SE DEPLACE VERS LA DEFAULT POSE")                                                                    
        
        # Donnees
        dof_idx = self.config.leg_joint2motor_idx 
        dof_size = len(dof_idx)
        done = False
        
        # record the current pos
        init_dof_pos = np.zeros(dof_size, dtype=np.float32)
        for i in range(dof_size):
            init_dof_pos[i] = self.low_state.motor_state[dof_idx[i]].q
    
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False
        self.count = 0
        while not done:
            self.count += 1

            if self.firstRun:
                for i in range(12):
                    self.startPos[i] = self.low_state.motor_state[i].q
                self.firstRun = False

            self.percent_1 += 1.0 / self.duration_1
            self.percent_1 = min(self.percent_1, 1)
            if self.percent_1 < 1:
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_1[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp =60
                    self.low_cmd.motor_cmd[i].kd = 5
                    self.low_cmd.motor_cmd[i].tau = 0

            if (self.percent_1 == 1) and (self.percent_2 <= 1):
                self.percent_2 += 1.0 / self.duration_2
                self.percent_2 = min(self.percent_2, 1)
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = (1 - self.percent_2) * self._targetPos_1[i] + self.percent_2 * self._targetPos_2[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = 60
                    self.low_cmd.motor_cmd[i].kd = 5
                    self.low_cmd.motor_cmd[i].tau = 0

            if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 < 1):
                self.percent_3 += 1.0 / self.duration_3
                self.percent_3 = min(self.percent_3, 1)
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] 
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = 60
                    self.low_cmd.motor_cmd[i].kd = 5
                    self.low_cmd.motor_cmd[i].tau = 0

            if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and (self.percent_4 <= 1):
                self.percent_4 += 1.0 / self.duration_4
                self.percent_4 = min(self.percent_4, 1)
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = (1 - self.percent_4) * self._targetPos_2[i] + self.percent_4 * self._targetPos_3[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = 60
                    self.low_cmd.motor_cmd[i].kd = 5
                    self.low_cmd.motor_cmd[i].tau = 0

            self.send_cmd(self.low_cmd)
            if self.percent_4 == 1.0 or self.count == 2500000000:
                done = True
            time.sleep(0.001)
        print("7] -------> LE ROBOT SE MAINTIENT DEBOUT")
        print("                                                       ")            
        print("            ###########################################")
        print("            # APPUYEZ SUR 'A' POUR DEMARRER LE MODELE #")
        print("            ###########################################")
        print("                                                       ") 
        while self.remote_controller.button[KeyMap.A] != 1:
            default = self.config.default_angles
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = default[i]
                self.low_cmd.motor_cmd[i].qd = 0
                self.low_cmd.motor_cmd[i].kp = 60
                self.low_cmd.motor_cmd[i].kd = 5
                self.low_cmd.motor_cmd[i].tau = 0
                self.send_cmd(self.low_cmd)
                time.sleep(0.002)
    ########################################################################################################################################

    ###################### Function that lowers the robot from a standing position ##################
    def move_to_ground(self):                                                                       #
        percent = 0                                                                                 #
        pos_init=[]                                                                                 #
        for k in range(12):                                                                         #
            pos_init.append(self.low_state.motor_state[k].q)                                        #
        while percent != 1:                                                                         #
            percent += 1.0 / 300                                                                    #
            percent = min(percent, 1)                                                               #
            couche = [0, 1.36, -2.65, 0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]          #
            for i in range(12):                                                                     #
                self.low_cmd.motor_cmd[i].q = (1 - percent) * pos_init[i] + percent * couche[i]     #
                self.low_cmd.motor_cmd[i].dq = 0                                                    #
                self.low_cmd.motor_cmd[i].kp = 60                                                   #
                self.low_cmd.motor_cmd[i].kd = 5                                                    #
                self.low_cmd.motor_cmd[i].tau = 0                                                   #
                self.send_cmd(self.low_cmd)                                                         #
            time.sleep(0.002)                                                                       #
        print("9] ---------> LE ROBOT EST ALLONGE")                                                 #
    #################################################################################################


    ######################### Function that estimates the robot’s instantaneous velocity from its joint data ##############################
    def compute_velocity(self,theta1,theta2,theta3,thetav1,thetav2,thetav3,foot):                                                         #
        vitessex = 0                                                                                                                      #
        vitessey = 0                                                                                                                      #
        vitessez = 0                                                                                                                      #
        l1 = 0.21                                                                                                                         #
        l2 = 0.23                                                                                                                         #
        num_pate = 0                                                                                                                      #
        for k in range(4):                                                                                                                #
            if foot[k] < 20:                                                                                                              #
                pate_au_sol = 0                                                                                                           #
            else:                                                                                                                         #
                pate_au_sol = 1                                                                                                           #
                num_pate += 1                                                                                                             #
            vitessex += pate_au_sol * ((-l1*sin(theta1[k])-l2*sin(theta1[k]+theta2[k]))*-thetav1[k] + (-l2*sin(theta1[k]+theta2[k]))*-thetav2[k]) 
            vitessey += pate_au_sol * ( (l1*cos(theta1[k])+l2*cos(theta1[k]+theta2[k]))*sin(theta3[k])*-thetav1[k]   +   (l2*cos(theta1[k]+theta2[k])*sin(theta3[k]))*-thetav2[k]  +  (l1*sin(theta1[k])+l2*sin(theta1[k]+theta2[k]))*cos(theta3[k])*thetav3[k]) 
            vitessez += pate_au_sol * ( -l1*cos(theta1[k])*thetav1[k] -l2*cos(theta1[k]+theta2[k])*(thetav1[k]+thetav2[k]))               #
        if num_pate > 0:                                                                                                                  #
            vitessex = vitessex / num_pate                                                                                                #
            vitessey = vitessey / num_pate                                                                                                #
            vitessez = vitessez / num_pate                                                                                                #
        return vitessex, vitessey, vitessez                                                                                               #
    #######################################################################################################################################


    # Main function that runs in a loop while the model is running
    def run(self):
        self.counter += 1

        ####### Recalibration of the data for velocity estimation ########
        theta1 = []                                                      #
        theta2 = []                                                      #
        theta3 = []                                                      #
        thetav1 = []                                                     #
        thetav2 = []                                                     #
        thetav3 = []                                                     #  
        theta1.append(-self.low_state.motor_state[1].q + 1.5708)         #    
        theta2.append(-self.low_state.motor_state[2].q - 1.7 + pi/2)     #
        theta3.append(-self.low_state.motor_state[0].q)                  #
        thetav1.append(self.low_state.motor_state[1].dq)                 #
        thetav2.append(self.low_state.motor_state[2].dq)                 #
        thetav3.append(-self.low_state.motor_state[0].dq)                #
                                                                         #
        theta1.append(-self.low_state.motor_state[4].q + 1.5708)         #  
        theta2.append(-self.low_state.motor_state[5].q - 1.7 + pi/2)     # 
        theta3.append(-self.low_state.motor_state[3].q)                  #
        thetav1.append(self.low_state.motor_state[4].dq)                 #
        thetav2.append(self.low_state.motor_state[5].dq)                 #
        thetav3.append(-self.low_state.motor_state[3].dq)                #  
                                                                         # 
        theta1.append(-self.low_state.motor_state[7].q + 1.5708)         # 
        theta2.append(-self.low_state.motor_state[8].q - 1.7 + pi/2)     #
        theta3.append(-self.low_state.motor_state[6].q)                  #
        thetav1.append(self.low_state.motor_state[7].dq)                 #
        thetav2.append(self.low_state.motor_state[8].dq)                 #
        thetav3.append(-self.low_state.motor_state[6].dq)                # 
                                                                         #
        theta1.append(-self.low_state.motor_state[10].q + 1.5708)        #
        theta2.append(-self.low_state.motor_state[11].q - 1.7 + pi/2)    #
        theta3.append(-self.low_state.motor_state[9].q)                  #
        thetav1.append(self.low_state.motor_state[10].dq)                # 
        thetav2.append(self.low_state.motor_state[11].dq)                #
        thetav3.append(-self.low_state.motor_state[9].dq)                #
                                                                         #
        foot = self.low_state.foot_force                                 #
        ##################################################################


        ############################ Vx #########################################################
        vx_calc = self.compute_velocity(theta1,theta2,theta3,thetav1,thetav2,thetav3,foot)[0]   #
        temp = 0                                                                                #
        for k in range(len(self.vx_window)):                                                    #
            temp += self.vx_window[k]                                                           #
        temp += vx_calc*5                                                                       #
        temp = temp/(len(self.vx_window)+5)                                                     #
                                                                                                #
        for k in range(len(self.vx_window)-1):                                                  #
            self.vx_window[k] = self.vx_window[k+1]                                             #
        self.vx_window[len(self.vx_window)-1] = temp                                            #
        vx = temp                                                                               #
        #########################################################################################


        ############################ Vy #########################################################
        vy_calc = self.compute_velocity(theta1,theta2,theta3,thetav1,thetav2,thetav3,foot)[1]   #
        temp = 0                                                                                #
        for k in range(len(self.vy_window)):                                                    #
            temp += self.vy_window[k]                                                           #
        temp += vy_calc*5                                                                       #
        temp = temp/(len(self.vy_window)+5)                                                     #
                                                                                                #
        for k in range(len(self.vy_window)-1):                                                  #
            self.vy_window[k] = self.vy_window[k+1]                                             #
        self.vy_window[len(self.vy_window)-1] = temp                                            #
        vy = temp                                                                               #
        #########################################################################################


        ############################ Vz #########################################################
        vz_calc = self.compute_velocity(theta1,theta2,theta3,thetav1,thetav2,thetav3,foot)[2]   #
        temp = 0                                                                                #
        for k in range(len(self.vz_window)):                                                    #
            temp += self.vz_window[k]                                                           #
        temp += vz_calc*5                                                                       #
        temp = temp/(len(self.vz_window)+5)                                                     #
                                                                                                #
        for k in range(len(self.vz_window)-1):                                                  #
            self.vz_window[k] = self.vz_window[k+1]                                             #
        self.vz_window[len(self.vz_window)-1] = temp                                            #
        vz = temp                                                                               #
        #########################################################################################


        ########################### Preparation of the data for the observations #######################################
        # Foot forces                                                                                                  #
        f0=self.low_state.foot_force[1]/100                                                                            #
        f1=self.low_state.foot_force[3]/100                                                                            #
        f2=self.low_state.foot_force[0]/100                                                                            #
        f3=self.low_state.foot_force[2]/100                                                                            #
                                                                                                                       #
        # Angle velocity                                                                                               #
        ang_vel = np.array([self.low_state.imu_state.gyroscope], dtype=np.float32)                                     #
                                                                                                                       #
        # Projected Gravitation                                                                                        #
        quat = self.low_state.imu_state.quaternion                                                                     #
        gravity_orientation = get_gravity_orientation(quat)                                                            #
                                                                                                                       #
        # Velocity commands                                                                                            #
        self.cmd[0] = round(self.remote_controller.ly,1)                                                               #
        self.cmd[1] = round(self.remote_controller.lx * -1,1)                                                          #
        self.cmd[2] = round(self.remote_controller.rx * -1,1)                                                          #
                                                                                                                       #
        # Collect the joint positions and velocities in the model’s order                                              #
        for i in range(len(self.config.leg_joint2motor_idx)):                                                          #
            self.qj[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].q                              #
            self.dqj[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].dq                            #
        qj_obs = self.qj.copy()                                                                                        #
        dqj_obs = self.dqj.copy()                                                                                      #
        defaut_joint = [0.1,-0.1,0.1,-0.1,0.8,0.8,1,1,-1.5,-1.5,-1.5,-1.5]                                             #
        qj_obs = qj_obs - defaut_joint                                                                                 #
        dqj_obs = dqj_obs                                                                                              #
                                                                                                                       #
        # The number of actions (12 for the Go2) plus 1 for the duration counter (Decimation * 0.005 ms)               #
        num_actions = self.config.num_actions                                                                          #
        count = self.counter * self.config.control_dt                                                                  #
        ################################################################################################################



        ########################################## Neural Network inputs :######## ############################################
        self.obs[:4]= [f0,f1,f2,f3]                                                                                           #
                                                                                                                              #
                                                                                                                              #
        self.obs[4:7]= [vx*2,vy*2,vz*2] #TO BE USED IF THE KALMAN FILTER DOESN’T WORK (MY VELOCITY ESTIMATION)                #
        #self.obs[4:7]= [node_kalman.base_lin_vel_input[0],node_kalman.base_lin_vel_input[1],node_kalman.base_lin_vel_input[2]]#
                                                                                                                              #
                                                                                                                              #
        self.obs[7:10] = ang_vel                                                                                              #
        self.obs[10:13] = gravity_orientation                                                                                 #
        self.obs[13:16] = self.cmd * self.config.cmd_scale * self.config.max_cmd                                              #
        self.obs[16 : 16 + num_actions] = qj_obs                                                                              #
        self.obs[16 + num_actions : 16 + num_actions * 2] = dqj_obs                                                           #
        self.obs[16 + num_actions * 2 : 16 + num_actions * 3] = self.action                                                   #
        #######################################################################################################################


        ############ Computation of the actions by the policy ##############
        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)               #
        self.action = self.policy(obs_tensor).detach().numpy().squeeze()   #
        ####################################################################


        ################# Sending commands to the motors #######################
        # Transform action to target_dof_pos                                   #
        target_dof_pos = self.action * self.config.action_scale + defaut_joint #
                                                                               #
        # Build low cmd                                                        #
        for i in range(len(self.config.leg_joint2motor_idx)):                  #
            motor_idx = self.config.leg_joint2motor_idx[i]                     #
            self.low_cmd.motor_cmd[motor_idx].q = target_dof_pos[i]            # 
            self.low_cmd.motor_cmd[motor_idx].qd = 0                           #
            self.low_cmd.motor_cmd[motor_idx].kp =  40 # self.config.kps[i]    #
            self.low_cmd.motor_cmd[motor_idx].kd = 0.5 # self.config.kds[i]    #
            self.low_cmd.motor_cmd[motor_idx].tau = 0                          #
        self.send_cmd(self.low_cmd)                                            #
        ########################################################################


        ##################### Store the data to be displayed ########################
        self.L_base_vel_cmd_input_1.append(self.obs[13])                            #
        self.L_base_vel_cmd_input_2.append(self.obs[14])                            #
        self.L_base_vel_cmd_input_3.append(self.obs[15])                            #
                                                                                    #
        self.L_base_lin_vel_input_1.append(vx*2)                                    #
        self.L_base_lin_vel_input_2.append(vx*2)                                    #
        self.L_base_lin_vel_input_3.append(vx*2)                                    #
                                                                                    #
        self.L_base_lin_vel_kalman_input_1.append(node_kalman.base_lin_vel_input[0])#
        self.L_base_lin_vel_kalman_input_2.append(node_kalman.base_lin_vel_input[1])#
        self.L_base_lin_vel_kalman_input_3.append(node_kalman.base_lin_vel_input[2])#
        self.L_base_lin_vel_kalman_input_4.append(node_kalman.base_lin_vel_input[3])#
                                                                                    #
        self.L_base_ang_vel_input_1.append(self.obs[3])                             #
        self.L_base_ang_vel_input_2.append(self.obs[4])                             #
        self.L_base_ang_vel_input_3.append(self.obs[5])                             #
        #############################################################################

        return self.obs


if __name__ == "__main__":
    import argparse

    ################################ Main function: preparation for the main loop ######################################
    parser = argparse.ArgumentParser()                                                                                 #
    parser.add_argument("net", type=str, help="network interface")                                                     #
    parser.add_argument("config", type=str, help="config file name in the configs folder", default="go2.yaml")         #
    args = parser.parse_args()                                                                                         #
                                                                                                                       #
    # Load config                                                                                                      #
    config_path = f"configs/{args.config}"                                                                             #
    config = Config(config_path)                                                                                       #
    print("1] -> LE FICHIER DE CONFIG A BIEN ETE CHARGE")                                                              #
                                                                                                                       #
    # Initialize DDS communication                                                                                     #
    ChannelFactoryInitialize(0, args.net)                                                                              #
    print("2] --> LE CHANNELFACTORY A ETE CREE")                                                                       #
                                                                                                                       #
    controller = Controller(config)                                                                                    #
    controller.Init()                                                                                                  #
                                                                                                                       #
    # Enter the zero torque state, press the start key to continue executing                                           #
    controller.zero_torque_state()                                                                                     #
                                                                                                                       #
    # Move to the default position                                                                                     #
    controller.move_to_default_pos()                                                                                   #
    ####################################################################################################################



    #################### Main loop of the model ######################################
    time_ms = 0  # Initial time                                                      #
    Liste_t = []                                                                     #
    print("8] --------> LE MODELE EST LANCE")                                        #
    print("                                                            ")            #
    print("             ###############################################")            #
    print("             # APPUYEZ SUR 'SELECT' POUR ARRETER LE MODELE #")            #
    print("             ###############################################")            #
    print("                                                            ")            #
                                                                                     #
    while True:                                                                      #
                                                                                     #
        try:                                                                         #
            obs = controller.run()                                                   #
            time.sleep(0.025)                                                        #
                                                                                     #
            Liste_t.append(time_ms)                                                  #
            time_ms += 25  # Increment of 25 ms                                      #
                                                                                     #
            # Press the select key to exit                                           #
            if controller.remote_controller.button[KeyMap.select] == 1:              #
                # Move to a laid position                                            #
                controller.move_to_ground()                                          #
                break                                                                #
                                                                                     #
        except KeyboardInterrupt:                                                    #
            break                                                                    #
    ##################################################################################


    ############### Visualization of data at the end of the experiment #######################
    print("10] ----------> VISUALISATION DES DONNEES EN COURS")                              #
                                                                                             #
    # On crée une grille 4 lignes × 2 colonnes                                               #
    fig = plt.figure(figsize=(24, 24))                                                       #
    gs = fig.add_gridspec(2, 2)                                                              #
                                                                                             #
    ax1 = fig.add_subplot(gs[0, 0])                                                          #
    ax2 = fig.add_subplot(gs[0, 1])                                                          #
    ax3 = fig.add_subplot(gs[1, 0])                                                          #
    ax4 = fig.add_subplot(gs[1, 1])                                                          #
    # --- 1. Vx                                                                              #
    ax1.plot(controller.L_base_vel_cmd_input_1, label="L_base_vel_cmd_input_1")              #
    ax1.plot(controller.L_base_lin_vel_input_1, label="L_base_lin_vel_input_1")              #
    ax1.plot(controller.L_base_lin_vel_kalman_input_1, label="L_base_lin_vel_kalman_input_1")#
    ax1.legend()                                                                             #
    ax1.set_title("Vx")                                                                      #
                                                                                             #
    # --- 2. Vy                                                                              #
    ax2.plot(controller.L_base_vel_cmd_input_2, label="L_base_vel_cmd_input_2")              #
    ax2.plot(controller.L_base_lin_vel_input_2, label="L_base_lin_vel_input_2")              #
    ax2.plot(controller.L_base_lin_vel_kalman_input_2, label="L_base_lin_vel_kalman_input_2")#
    ax2.legend()                                                                             #
    ax2.set_title("Vy")                                                                      #
                                                                                             #
    # --- 3. Vz                                                                              #
    ax3.plot(controller.L_base_lin_vel_input_3, label="L_base_lin_vel_input_3")              #
    ax3.plot(controller.L_base_lin_vel_kalman_input_3, label="L_base_lin_vel_kalman_input_3")#
    ax3.legend()                                                                             #
    ax3.set_title("Vz")                                                                      #
                                                                                             #
    # --- 4. Wz                                                                              #
    ax4.plot(controller.L_base_vel_cmd_input_3, label="L_base_vel_cmd_input_3")              #
    ax4.plot(controller.L_base_ang_vel_input_1, label="L_base_ang_vel_input_1")              #
    ax4.plot(controller.L_base_lin_vel_kalman_input_4, label="L_base_lin_vel_kalman_input_4")#
    ax4.legend()                                                                             #
    ax4.set_title("Wz")                                                                      #
                                                                                             #
                                                                                             #
    plt.tight_layout()                                                                       #
    plt.savefig("analyse_robot.pdf") # Save in PDF                                           #
    plt.show()                                                                               #
    ##########################################################################################

    print("EXIT")
