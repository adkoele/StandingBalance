"""This file implements reflex controller."""
# Default modules
import os

import numpy as np
import biolog
import yaml

import optimization.webots as optimization
from datalogger.DataLogger import DataLogger
from time_delay_states.time_delay_states import time_delay_states
from controller import Supervisor, Node
from musculo_skeletal_generator.musculo_skeletal_generator import MusculoSkeletalSystem
from webots_communication import webots_communication
from body_communication import body_communication
from parameters import parameters
from objectives import objectives
from reflex_controller import balance_controller

class Human(Supervisor):
    """Main class for Human control. """

    def __init__(self):
        super(Human, self).__init__()
        self.TIMESTEP = int(self.getBasicTimeStep())
        
        self.logger = biolog.Logger()
        self.time_elapsed = 0.
        self.largest_time_delay = 51;
        
        self.webots_communication = None
        self.body_communication = None
        self.musculo_sys = None
        
        #: Initialize webots communication
        self.webots_communication = webots_communication()
        self.body_communication = body_communication()
        self.body_communication.state_machine = 0
        self.anklemuscles = ['tib_ant_r', 'tib_ant_l', 'soleus_r', 'soleus_l', 'med_gas_l', 'med_gas_r']
        self.kneemuscles = ['med_gas_l', 'med_gas_r', 'bifemlh_l', 'bifemlh_r', 'rec_fem_r', 'rec_fem_l',
            'bifemsh_l', 'bifemsh_r', 'vas_int_r', 'vas_int_l']
        self.hipmuscles = ['psoas_r', 'psoas_l', 'glut_max_r', 'glut_max_l', 'bifemlh_l', 'bifemlh_r',
            'rec_fem_r', 'rec_fem_l']
            
        #: Change this to switch between controllers
        case = 'normal' # Options: 'normal' 'force'  'length'  'TD'
        
        #: Initialize musculoskeletalsystem
        #: Here, you might have to change the path to the controller file
        if case == 'TD':
            self.musculo_sys = self.body_communication.initialize_musculoskeletal_system('../Configuration/human-config-TDfromlitmin10.yaml')
        else:
            self.musculo_sys = self.body_communication.initialize_musculoskeletal_system('../Configuration/human-config-TDfromlit.yaml')
        
        # Create arrays of names
        nodes_names = ['trunk', 'hip_l' ,'hip_r','knee_l','knee_r','ankle_l','ankle_r']
        bodies_names = ['trunk', 'thigh_l', 'thigh_r', 'shin_l', 'shin_r', 'foot_l', 'foot_r']
        nodes_def = ['TRUNK', 'LEFT_HIP', 'RIGHT_HIP', 'LEFT_KNEE', 'RIGHT_KNEE', 'LEFT_FOOT', 'RIGHT_FOOT']
        bodies_def = ['REGIS', 'LEFT_THIGH', 'RIGHT_THIGH', 'LEFT_SHIN', 'RIGHT_SHIN', 'LEFT_FOOT', 'RIGHT_FOOT']
        contact_names = ['HEEL_LEFT', 'HEEL_RIGHT', 'TOE_LEFT', 'TOE_RIGHT']
        
        self.webots_communication.initialize_robot(self, contact_names, nodes_names, bodies_names, nodes_def, bodies_def)
        self.webots_communication.update()
        self.body_communication.initialize_body(self.musculo_sys, self.webots_communication, self.largest_time_delay)
        
        # Uncomment all log_data lines to log data
        #self.log_data = DataLogger('Raw_files/', self.body_communication, ["angles", "moments"], ["act", "stim", "l_ce", "f_se"])
        #self.log_data.counter = 1001
               
        # Initialize optimization not needed now
        self.opti = 0
        #self.opti = optimization.Webots()
          
        self.objectives = objectives('minimize_passive_torques', 'minimize_activation', 
            'balance', 'fb_activation', 'ankle_pos')
        
        # Normal
        if case == 'normal':
            parameter_file = '../Configuration/BaseModel.yaml'
        elif case == 'length':
            # Length only
            parameter_file = '../Configuration/LengthFeedbackModel.yaml'
        elif case == 'force':
            # Force only
            parameter_file = '../Configuration/ForceFeedbackModel.yaml'
        elif case == 'TD':
            # TD
            parameter_file = '../Configuration/BaseModel_TD.yaml'
        
        self.parameters = parameters()
        self.parameters.initialize_parameters(parameter_file, self.body_communication, 'balance',1)
                        
        self.load_parameters(parameter_file)
        
        self.controller = balance_controller(com = True)
                
        if self.opti == 0:
            self.controller.controller_params(self.body_communication.muscles, self.parameters.cparams, 0)
        else:
            self.controller.controller_params(self.body_communication.muscles, self.parameters.cparams, self.opti.parameters)
                
        # Give initial activation state
        if self.opti != 0:
            for name in self.anklemuscles:#body_communication.muscles:
                self.musculo_sys.dae.x.set_val('A_' + name, self.opti.parameters[name[:-2]].value)
                
        for name in self.body_communication.muscles:
            self.controller.mn[name] = 0.
            
    def run(self):
        """ Run """
        self.time_elapsed = 0.
        stop = 0
        
        angles = {}
        angles['trunk'] = 0.05 
        angles['knee'] = 0.18730316343363179188
        angles['hip'] = -0.171961183386423355211
        angles['ankle'] = self.find_ankle_angle(angles) 
        
        init_anklepos = self.webots_communication.nodes['ankle_l'].getPosition()[2]

        while self.step(self.TIMESTEP) != -1: 
            self.time_elapsed += self.TIMESTEP / 1000.0;
            
            if self.time_elapsed < 1:
                self.periodicity = 0
            
            #Update webots and body (musculoskeletal system and network)
            self.webots_communication.update()
            self.body_communication.update(self.musculo_sys, self.webots_communication, self.controller.mn)

            #: Update muscle activations
            self.controller.balance_controller_fb(self.body_communication.muscles, self.webots_communication, self.body_communication)
            
            #: Update activations in muscle system
            for name in self.body_communication.muscles: 
                self.musculo_sys.muscle_sys.activations [name] = self.controller.activations[name]
                    
            for name, motor in self.webots_communication.motors.iteritems():
                if self.time_elapsed < 1:
                    motor.setPosition(angles[name[:-2]])
                else:
                    motor.setTorque(self.body_communication.new_torque[name])
            
            #: Add the new states to the logger --> uncomment for logging
            #self.log_data.add_to_logger(self.body_communication, self.webots_communication.ground_reaction_force,
            #    self.webots_communication.ground_contacts, False, 10)
            
            #: Calculate objectives
            self.objectives.calc_minimize_passive_torques(self.body_communication)
            self.objectives.calc_minimize_activation(self.body_communication)  
                
            self.objectives.calc_objective_balance(self.webots_communication)
            self.objectives.calc_ankle_pos(self.webots_communication, init_anklepos)
            
            if self.time_elapsed < 10:
                self.objectives.calc_objective_fb_min(self.body_communication, self.controller.mn)
            
            #: Decide about termination
            pos_trunk = self.webots_communication.trunk_pos                    
            if pos_trunk[1] < 0.8:
                stop = 1
                
            if self.time_elapsed > 310:#110:#32:#5:#
                stop = 1
              
            if stop == 1:
                self.objectives.divide_by_time(self.time_elapsed*1000., 'minimize_passive_torques', 'minimize_activation', 'ankle_pos')
                self.objectives.divide_by_time(10*1000., 'fb_activation')
                
                #: Evaluate the fitness and return to optimizer, quit
                if self.opti != 0:
                    self.opti.respond({'time_elapsed': self.time_elapsed, 'hip_pos': pos_trunk[1],
                        'obj_value': self.objectives.objectives['balance'], 'activation': 
                        self.objectives.objectives['minimize_activation'], 'soft_limits': 
                        self.objectives.objectives['minimize_passive_torques'], 'act_fb':
                        self.objectives.objectives['fb_activation'], 'ankle_pos': self.objectives.objectives['ankle_pos']}) # 'periodicity': periodicity/self.time_elapsed, 
                # Uncomment for logging, comment line: self.simulationQuit(0)
                #self.log_data.write_to_files(self.body_communication)
                #self.simulationSetMode(0)
                #self.worldReload()
                self.simulationQuit(0)

    def find_ankle_angle(self, angles):
        # function to calculate the ankle angle such that the COM is in the middle of the foot
        
        #: Calculate desired location
        DES_BoS = self.webots_communication.get_desired_bos()
        CoM_perbody = self.webots_communication.get_center_of_mass_parts()
        
        sin_angle_tot = DES_BoS-(np.sin(angles['trunk'])*CoM_perbody['trunk'][2]-np.sin(angles['trunk']+angles['hip'])
            *CoM_perbody ['thigh_l'][2]-np.sin(angles['trunk']+angles['hip']+angles['knee'])*CoM_perbody['shin_l'][2]
            - np.sin(angles['trunk'] +angles['hip'])*CoM_perbody['thigh_r'][2]-np.sin(angles['trunk']+angles['hip']+
            angles['knee'])*CoM_perbody['shin_r'][2])/(CoM_perbody['foot_r'][2]+CoM_perbody['foot_l'][2])
        ankle = np.arcsin(sin_angle_tot) - (angles['trunk']+angles['hip']+angles['knee'])
        if np.isnan(ankle):
            ankle = angles['ankle']
        return ankle
        
    def load_parameters(self,control_path):
        try:
            stream = file(os.path.realpath(control_path), 'r')
            self.cparams = yaml.load(stream)
            '''biolog.info('Successfully loaded the file : {}'.format(
                os.path.split(control_path)[-1]))'''
            return
        except ValueError:
            biolog.error('Unable to read the file {}'.format(control_path))
            raise ValueError()

def main():
    """ Main """
    human = Human()
    human.run()

if __name__ == '__main__':
    main()

