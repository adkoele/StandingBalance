import os

import numpy as np
import biolog
import yaml
from collections import OrderedDict

import optimization.webots as optimization
from datalogger.DataLogger import DataLogger
from time_delay_states.time_delay_states import time_delay_states
from controller import Supervisor, Node
from musculo_skeletal_generator.musculo_skeletal_generator import MusculoSkeletalSystem

class body_communication(object):
    '''This class is used to communicate with the body: the states of the 
    multibody dynamics and muscle model are updated here. It can be
    extended by also saving parameters here.'''
    
    def __init__(self, state_machine = 0):
        self.weight = 80.*9.81
        #: Joint variables
        self.joints = {}
        self.joint_positions = {}
        self.joint_velocities = {}
        self.prev_joint_positions = {}
        self.prev_joint_velocities = {}
        self.head_position = {}
        self.head_prev_position = {}
        self.head_velocity = {}
        
        #: Muscle variables
        self.muscles = {}
        self.l_ce = {}
        self.act = {}
        self.f_se = {}
        self.f_pe = {}
        self.f_ce = {}
        self.stim = {}
        self.v_ce = {}
        self.prev_act= {}
        self.prev_lce= {}
        self.prev_stim= {}
        
        #: Muscle parameters
        self.l_slack = {}
        self.l_opt = {}
        self.v_max = {}
        self.f_max = {}
        self.td_to_sc = {}
        self.td_from_sc = {}
        self.pennation = {}
        self.motiontype = {}
        self.muscle_joints = OrderedDict()
        self.muscle_inhibitions = {}
        
        #: Torque variables
        self.new_torque = {}
        self.torque_soft_limit = {}
        
        #: Network variables
        self.mn = {}
        
        #: State machine variables
        self.state_machine = state_machine
        if self.state_machine == 1:
            self.stance = {}
            self.swing = {}
            self.DS = {}
            self.qto = {}
            self.num_strides = 0
            self.num_steps = {}
            self.num_steps['L'] = 0
            self.num_steps['R'] = 0
            self.didTouchTheGround = False
        self.periodicity_joint_positions = {}
        self.periodicity_joint_velocities = {}
        self.periodicity_act = {}
        self.periodicity_l_ce = {}
        self.periodicity_stim = {}
        self.periodicity = 0.        
        self.update_periodicity = 0
        
    def initialize_body(self, musculo_sys, webots_communication, largest_time_delay = 20):
        # Create time delayed states
        self.initialize_joint_positions(musculo_sys, webots_communication)
        self.initialize_muscle_states(musculo_sys)
        if self.state_machine == 1:
            self.initialize_state_machine(musculo_sys, webots_communication.nodes)
        self.initialize_periodicity(musculo_sys)
        self.delay_states = time_delay_states(self, webots_communication, largest_time_delay)
        
        #Add muscle parameters
        for muscle in self.muscles:
            dae = musculo_sys.muscle_sys.muscles[muscle].dae
            
            if dae != 0:
                self.l_slack[muscle] = musculo_sys.dae.c.get_val('l_slack_'+muscle)
                self.l_opt[muscle] = musculo_sys.dae.c.get_val('l_opt_'+muscle)
                self.v_max[muscle] = musculo_sys.dae.c.get_val('v_max_'+muscle)
                self.f_max[muscle] = musculo_sys.dae.c.get_val('f_max_'+muscle)
                self.td_to_sc[muscle] = musculo_sys.dae.c.get_val('td_to_sc_'+muscle)
                self.td_from_sc[muscle] = musculo_sys.dae.c.get_val('td_from_sc_'+muscle)
                self.pennation[muscle] = musculo_sys.dae.c.get_val('pennation_'+muscle)
                '''if self.musculo_sys.dae.c.get_val('motiontype_'+muscle) == 1:
                    self.motiontype[muscle] = 'flexor'
                else:
                    self.motiontype[muscle] = 'extensor'''      
            else:
                self.l_slack[muscle] = musculo_sys.muscle_sys.muscles[muscle]._lsee_slack
                self.l_opt[muscle] = musculo_sys.muscle_sys.muscles[muscle]._l_opt
                self.v_max[muscle] = musculo_sys.muscle_sys.muscles[muscle]._v_max
                self.f_max[muscle] = musculo_sys.muscle_sys.muscles[muscle]._f_max
                self.td_to_sc[muscle] = musculo_sys.muscle_sys.muscles[muscle].td_to_sc
                self.td_from_sc[muscle] = musculo_sys.muscle_sys.muscles[muscle].td_from_sc
                self.pennation[muscle] = musculo_sys.muscle_sys.muscles[muscle]._pennation

        #Add muscle joints
        for muscle in self.muscles:
            self.muscle_joints[muscle] = {}
            for joint in musculo_sys.muscle_sys.muscles[muscle].muscle_joints:
                self.muscle_joints[muscle][joint.joint.name] = joint.joint.name
                
        #print(self.muscle_joints)
        # Add muscles that it inhibits from
        for muscle in self.muscles:
            # loop through joints that this muscle is connected to
            for joint in self.muscle_joints[muscle]:
                # loop through all muscles
                for muscle1 in self.muscles:
                    # only check if the muscle is a different one
                    if muscle1 != muscle:
                        # loop through joints of other muscles
                        for joint1 in self.muscle_joints[muscle1]:
                            # check if this joint is the same as the joint of the current muscle
                            if joint1 == joint:
                                self.muscle_inhibitions[muscle1] = muscle1
                        
    #Initialization functions
    def initialize_musculoskeletal_system(self,config_file = '../../../modeling/configFiles/human-config.yaml'):
        """
        Initialize musculoskeletalsystem.
        """
        biolog.info('Initializing musculoskeletalsystem')
        dt = 0.001
        opts = {'tf': dt,
                'jit': False,
                "enable_jacobian": True,
                "print_time": False,
                "print_stats": False,
                "reltol": 1e-6,
                "abstol": 1e-6} 

        musculo_sys = MusculoSkeletalSystem(config_file, opts=opts)
        return musculo_sys
        
    def initialize_joint_positions(self, musculo_sys, webots_communication):
        '''
        Initialize the joint positions and velocities of musculoskeletal system from webots.
        '''
        for name in musculo_sys.joint_sys.joints.iterkeys():
            self.joints[name] = name
            self.joint_positions[name] = webots_communication.position_sensors[name].getValue()
            self.joint_velocities[name] = 0.

        trunk = webots_communication.nodes['trunk'].getOrientation()
        self.joint_positions['trunk'] = np.arccos(trunk[4])
        self.joint_velocities['trunk'] = 0.
        self.head_position['x'] = 0.
        self.head_position['y'] = 0.
        self.head_prev_position['x'] = 0.
        self.head_prev_position['y'] = 0.
        self.head_velocity['x'] = 0.
        self.head_velocity['y'] = 0.
            
    def initialize_muscle_states(self, musculo_sys):
        for name in musculo_sys.muscle_sys.muscles.iterkeys():
            self.muscles[name] = name
            self.l_ce[name] = musculo_sys.muscle_sys.muscles[name].l_ce
            self.act[name] = musculo_sys.muscle_sys.muscles[name].act
            self.f_se[name] = musculo_sys.muscle_sys.muscles[name].f_se
            self.f_pe[name] = musculo_sys.muscle_sys.muscles[name].f_pe
            self.f_ce[name] = musculo_sys.muscle_sys.muscles[name].f_ce
            self.stim[name] = musculo_sys.muscle_sys.activations[name]   
            self.v_ce[name] = musculo_sys.muscle_sys.muscles[name].v_ce
            
    def initialize_state_machine(self, musculo_sys, nodes):
        self.stance['R'] = 0
        self.stance['L'] = 0
        self.swing['R'] = 0
        self.swing['L'] = 0
        self.DS['R'] = 1
        self.DS['L'] = 1
        
        self.qto['R'] = self.joint_positions['trunk']
        self.qto['L'] = self.joint_positions['trunk']
        
    def initialize_periodicity(self, musculo_sys):
        for joint in musculo_sys.joint_sys.joints.iterkeys():
            self.periodicity_joint_positions[joint] = 0
            self.periodicity_joint_velocities[joint] = 0
            
        for muscle in musculo_sys.muscle_sys.muscles.iterkeys():
            self.periodicity_act[muscle] = 0
            self.periodicity_l_ce[muscle] = 0
            self.periodicity_stim[muscle] = 0
      
    #Update functions
    def update(self, musculo_sys, webots_communication, mn):
        '''
        Update the musculoskeletal system: run the dynamics and store variables
        '''
        
        #: Step the joint positions
        self.update_joint_positions(webots_communication)
        
        #: Step the time delay states
        self.update_delay_states(webots_communication, mn)
        
        #: Step the musculo_skeletal_system
        musculo_sys.step(self.joint_positions)
        
        #: Step the state machine
        if self.state_machine == 1:
            self.step_state_machine(webots_communication)
            #print("L, DS: {}, Swing: {}, Stance: {}".format(self.DS['L'], self.swing['L'], self.stance['L']))
            #print("R, DS: {}, Swing: {}, Stance: {}".format(self.DS['R'], self.swing['R'], self.stance['R']))

                                
        #: Store variables
        for name in self.joints:
            self.new_torque[name] = musculo_sys.joint_sys.joints[name].torque
            self.torque_soft_limit [name] = musculo_sys.joint_sys.joints[name].torque_soft_limit
        for name in self.muscles:
            self.l_ce[name] = musculo_sys.muscle_sys.muscles[name].l_ce
            self.act[name] = musculo_sys.muscle_sys.muscles[name].act
            self.f_se[name] = musculo_sys.muscle_sys.muscles[name].f_se
            self.f_pe[name] = musculo_sys.muscle_sys.muscles[name].f_pe
            self.f_ce[name] = musculo_sys.muscle_sys.muscles[name].f_ce
            self.stim[name] = musculo_sys.muscle_sys.activations[name]   
            self.v_ce[name] = musculo_sys.muscle_sys.muscles[name].v_ce
            
    def update_joint_positions(self, webots_communication):
        '''
        Update the joint positions of musculoskeletal system from webots.
        '''
        for name in webots_communication.position_sensors:
            self.prev_joint_positions[name] = self.delay_states.get_angle(1,name)
            self.joint_positions[name] = webots_communication.position_sensors[name].getValue()
            
        self.prev_joint_positions['trunk'] = self.joint_positions['trunk']
        trunk = webots_communication.nodes['trunk'].getOrientation()
        self.joint_positions['trunk'] = np.arccos(trunk[4])
        
        for name in self.joint_velocities:
            self.joint_velocities[name] = (self.joint_positions[name] - self.prev_joint_positions[name])*1000.
        
        self.head_position['x'] = np.sin(self.joint_positions['trunk'])*0.4 + webots_communication.trunk_pos[2]
        self.head_velocity['x'] = (self.head_position['x'] - self.head_prev_position['x'])*1000.
        self.head_prev_position['x'] = self.head_position['x']
        self.head_position['y'] = trunk[4]*0.4 + webots_communication.trunk_pos[1]
        self.head_velocity['y'] = (self.head_position['y'] - self.head_prev_position['y'])*1000.
        self.head_prev_position['y'] = self.head_position['y']
        self.head_position['s'] = np.sqrt(np.square(self.head_position['x'])+np.square(self.head_position['y']))
        self.head_velocity['s'] = np.sqrt(np.square(self.head_velocity['x'])+np.square(self.head_velocity['y']))
        
    def update_delay_states(self, webots_communication, mn):
        self.delay_states.step(self, webots_communication, mn)
        
    def step_state_machine(self, webots_communication):    
        if(not self.didTouchTheGround and webots_communication.ground_reaction_force['Ly'] > 0.1 and webots_communication.ground_reaction_force['Ry'] > 0.1):
            self.didTouchTheGround = True

        if(not self.didTouchTheGround):
            return
        #: Right leg is in stance
        if self.stance['R'] == 1 and self.swing['L'] == 1:
            self.DS['R'] = 0
            self.swing['R'] = 0
            self.stance['L'] = 0
            self.DS['L'] = 0
            
            # Check if double support starts
            if webots_communication.ground_reaction_force['Ly'] > 0.1: #new step left
                self.stance['R'] = 0
                self.DS['R'] = 1
                self.swing['L'] = 0
                self.stance['L'] = 1
                self.num_steps['L'] += 1 
        elif self.DS['R'] == 1 and self.DS['L'] != 1:
            self.stance['R'] = 0
            self.swing['R'] = 0
            #check if swing starts
            if webots_communication.ground_reaction_force['Ry'] < 0.1:
                self.swing['R'] = 1
                self.DS['R'] = 0
                self.qto['R'] = self.delay_states.get_trunk_angle(5)
        elif self.stance['L'] == 1 and self.swing['R'] == 1:
            self.DS['L'] = 0
            self.swing['L'] = 0
            self.stance['R'] = 0
            self.DS['R'] = 0
            if webots_communication.ground_reaction_force['Ry'] > 0.1: #new step right and stride
                self.stance['L'] = 0
                self.DS['L'] = 1
                self.swing['R'] = 0                
                self.stance['R'] = 1
                self.num_steps['R'] += 1 
                self.num_strides += 1
                
                if self.num_strides == 5:
                    self.periodicity = 0 #disregard first strides
                    
                #Calculate difference with previous stride
                self.calculate_periodicity()
                self.update_periodicity_states()
                self.update_periodicity = 1;    
    
        elif self.DS['L'] == 1 and self.DS['R'] != 1:
            self.stance['L'] = 0
            self.swing['L'] = 0
            #check if swing starts
            if webots_communication.ground_reaction_force['Ly'] < 0.1:
                self.swing['L'] = 1
                self.DS['L'] = 0
                self.qto['L'] = self.delay_states.get_trunk_angle(5)
        else:
            # start of the simulation
            if webots_communication.ground_reaction_force['Ry'] > 0.1:
                self.swing['R'] = 0
                self.DS['R'] = 1
                self.stance['L'] = 1
                self.DS['L'] = 0
                self.qto['R'] = self.delay_states.get_trunk_angle(5)
            elif webots_communication.ground_reaction_force['Ly'] > 0.1:
                self.swing['L'] = 0
                self.DS['L'] = 1
                self.stance['R'] = 1
                self.DS['R'] = 0
                self.qto['L'] = self.delay_states.get_trunk_angle(5)    
    
    #Periodicity calculation and update
    def calculate_periodicity(self):
        for joint in self.joints:
            self.periodicity += np.square(self.joint_positions[joint]-self.periodicity_joint_positions[joint])
            self.periodicity += np.square(self.joint_velocities[joint]-self.periodicity_joint_velocities[joint])
            
        for muscle in self.muscles:
            self.periodicity += np.square(self.l_ce[muscle]-self.periodicity_l_ce[muscle])
            self.periodicity += np.square(self.act[muscle]-self.periodicity_act[muscle])
            self.periodicity += np.square(self.stim[muscle]-self.periodicity_stim[muscle])
             
    def update_periodicity_states(self):
        for joint in self.joints:
            self.periodicity_joint_positions[joint] = self.joint_positions[joint]
            self.periodicity_joint_velocities[joint] = self.joint_velocities[joint]
        
        for muscle in self.muscles:
            self.periodicity_l_ce[muscle] = self.l_ce[muscle]
            self.periodicity_act[muscle] = self.act[muscle]
            self.periodicity_stim[muscle] = self.stim[muscle] 
