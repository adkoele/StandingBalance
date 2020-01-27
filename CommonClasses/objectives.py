import numpy as np
from webots_communication import webots_communication
from body_communication import body_communication

class objectives():
    '''This class creates an object that is used for communication with
    webots
    '''
    #Default parameters

    def __init__(self, *objectives):
        '''This function initializes the objectives.
        '''
        self.objectives = {}
        #Declare variables
        for obj in objectives:
            self.objectives[obj] = 0.

    #General functions
    def divide_by_nsteps(self, num_steps, *objectives):
        if num_steps > 0:
            self.objectives['minimize_periodic_gait'] = self.objectives['minimize_periodic_gait']/num_steps
        else:
            self.objectives['minimize_periodic_gait'] = self.objectives['minimize_periodic_gait']*10
        
    def respond_to_optimizer(self,opti):
        opti.respond({'distance': self.objectives['maximize_distance'], 'speed': self.objectives['velocity'],
            'periodicity': self.objectives['minimize_periodic_gait'], 'activation': 
            self.objectives['minimize_activation'], 'soft_limits': self.objectives['minimize_passive_torques']})
            
    #Objective functions
    
    #Minimize passive torques    
    def calc_minimize_passive_torques(self, body_communication):
        for name in body_communication.joints:
            self.objectives['minimize_passive_torques'] += np.square(body_communication.torque_soft_limit[name])
        
    #Match desired velocity
    def calc_velocity(self, distance, time_elapsed):
        if np.abs(time_elapsed) < 1e-3:
            self.objectives['velocity'] = 20
        else:
            self.objectives['velocity'] = distance/time_elapsed
        
    #Minimize activation
    def calc_minimize_activation(self, body_communication):
        for name in body_communication.muscles:
            self.objectives['minimize_activation'] += np.square(body_communication.act[name])

    #Minimize stimulation
    def calc_minimize_stimulation(self, body_communication):
        for name in body_communication.muscles:
            self.objectives['minimize_stimulation'] += np.square(body_communication.stim[name])

    #Maximize distance walked
    def calc_maximize_distance(self, pos_trunk):
        self.objectives['maximize_distance'] = pos_trunk        
        
    #Create periodicity
    def calc_minimize_periodic_gait(self, periodicity):
        self.objectives['minimize_periodic_gait'] += periodicity
        
    def divide_by_time(self, time_elapsed, *objectives):
        print(time_elapsed)
        for name in objectives:
            if np.abs(time_elapsed) < 1e-3:
                self.objectives[name] = self.objectives[name]*1000
            else:
                self.objectives[name] = self.objectives[name]/time_elapsed
            
    def calc_ankle_pos(self, webots_communication, initpos = 0.):
        pos_l = webots_communication.nodes['ankle_l'].getPosition()[2]
        pos_r = webots_communication.nodes['ankle_r'].getPosition()[2]
        self.objectives['ankle_pos'] += np.abs(pos_l - initpos)/2.
        self.objectives['ankle_pos'] += np.abs(pos_r - initpos)/2.
            
    #Balance
    def calc_objective_balance(self, webots_communication):
        #: Calculate center of pressure
        CoP = webots_communication.get_center_of_pressure()
        #: Calculate center of mass
        CoM = webots_communication.get_center_of_mass()
        
        #: Calculate base of support
        DES_BoS = webots_communication.get_desired_bos()
        
        self.objectives['balance'] = np.abs(CoP-DES_BoS) + np.abs(CoM[2]-DES_BoS)

    #Minizie feedback controld
    def calc_objective_fb_min(self, body_communication, mn):
        for name in body_communication.muscles:
            self.objectives['fb_activation'] += np.square(mn[name])
