''' All function related to communication with webots'''
import os

import numpy as np
import biolog
import yaml

import optimization.webots as optimization
from datalogger.DataLogger import DataLogger
from controller import Supervisor, Node
from musculo_skeletal_generator.musculo_skeletal_generator import MusculoSkeletalSystem

class webots_communication(object):
    '''This class creates an object that is used for communication with
    webots
    '''
    #Default parameters

    def __init__(self):
        '''This function initializes the lists required for  webots communcation.
        '''
        #Declare variables
        self.nodes = {}
        self.bodies = {}
        self.ground_contacts = {}
        self.ground_contact_positions = {}
        self.ground_contact_nodes = {}
        self.ground_contact_sensors = {}
        self.ground_reaction_force = {}
        self.motors = {}
        self.position_sensors = {}
        self.trunk_pos = []
        
    def initialize_robot(self,robot, contact_names, nodes_names, bodies_names, nodes_def, bodies_def):
        '''This function initializes the webots communications:
        Inputs: robot: the webots supervisor
                contact_names: the names of the contact points for grf sensing
                nodes_names: the internal name of the nodes for which angles are sensed
                bodies_names: the internal name of the bodies for which CoM is determined
                nodes_def: the name of the nodes in the webots world
                bodies_def: the name of the bodies in the webots world 
        '''
        #Initialize variables
        self.initialize_webots_nodes(robot, nodes_names, bodies_names, nodes_def, bodies_def)
        self.initialize_webots_motors(robot)
        self.initialize_webots_position_sensors(robot)
        self.initialize_webots_ground_contact_sensors(robot, contact_names)
        
        self.trunk_pos = self.nodes['trunk'].getPosition()
        
    #Initialization functions
    def initialize_webots_nodes(self, robot, nodes_names, bodies_names, nodes_def, bodies_def):
    #    '''Set-up leg joints in the system.'''

        for i in range (0,len(nodes_names)):
            self.nodes[nodes_names[i]] = robot.getFromDef(nodes_def[i])
            
        for i in range (0,len(bodies_names)):
            self.bodies[bodies_names[i]] = robot.getFromDef(bodies_def[i])
        
    def initialize_webots_motors(self, robot):
        '''Set-up leg joints in the system.'''
        for joint in robot.musculo_sys.joint_sys.joints.iterkeys():
            self.motors[joint] = robot.getMotor(str(joint))

    def initialize_webots_position_sensors(self, robot):
        '''Set-up leg joints in the system.'''
        for joint in robot.musculo_sys.joint_sys.joints.iterkeys():
            self.position_sensors[joint] = robot.getPositionSensor(
                str(joint) + '_POS')
            self.position_sensors[joint].enable(robot.TIMESTEP)

    def initialize_webots_ground_contact_sensors(self, robot, contact_names):
        '''Initialize groung contact sensors.'''
        for name in contact_names:
            self.ground_contact_sensors['SENSOR_'+name] = robot.getTouchSensor('SENSOR_'+name)
            self.ground_contact_sensors['SENSOR_'+name].enable(robot.TIMESTEP)
            
            self.ground_contact_nodes[name] = robot.getFromDef('SENSOR_'+name)
        
        #for name in self.ground_contact_nodes:
            self.ground_contacts[name] = self.ground_contact_sensors['SENSOR_'+name].getValues()

    #Function to update sensors
    def update(self):
        self.update_ground_contact_sensors()
        self.update_ground_contacts()
        self.trunk_pos = self.nodes['trunk'].getPosition()
        
    def update_ground_contact_sensors(self):
        for sensor in self.ground_contacts:
            self.ground_contacts[sensor] = self.ground_contact_sensors['SENSOR_'+sensor].getValues()
            self.ground_contact_positions[sensor] = self.ground_contact_nodes[sensor].getPosition()
        
    def update_ground_contacts(self):
        ''' Update ground contacts '''
        self.update_ground_contact_sensors()
        
        ground_contacts_left = self.ground_contacts['TOE_LEFT']+self.ground_contacts['HEEL_LEFT']
        ground_contacts_right = self.ground_contacts['TOE_RIGHT']+self.ground_contacts['HEEL_RIGHT']
        
        self.ground_reaction_force['Lx'] = ground_contacts_left[1] + ground_contacts_left[4]
        self.ground_reaction_force['Ly'] = -ground_contacts_left[2] - ground_contacts_left[5]
        self.ground_reaction_force['Rx'] = ground_contacts_right[1] + ground_contacts_right[4]
        self.ground_reaction_force['Ry'] = -ground_contacts_right[2] - ground_contacts_right[5]

        self.ground_reaction_force['Lx'] = 1/(1+np.exp(-5*(self.ground_reaction_force['Lx']-100)))*self.ground_reaction_force['Lx'] # 
        self.ground_reaction_force['Ly'] = 1/(1+np.exp(-5*(self.ground_reaction_force['Ly']-100)))*self.ground_reaction_force['Ly'] # -ground_contacts_left[2]
        self.ground_reaction_force['Rx'] = 1/(1+np.exp(-5*(self.ground_reaction_force['Rx']-100)))*self.ground_reaction_force['Rx'] # ground_contacts_right2[2]
        self.ground_reaction_force['Ry'] = 1/(1+np.exp(-5*(self.ground_reaction_force['Ry']-100)))*self.ground_reaction_force['Ry'] # 

    # Functions to get information from webots
    def get_center_of_mass_parts(self):
        CoM_parts = {}
        
        for body in self.bodies:
            CoM_parts[body] = self.bodies[body].getCenterOfMass()
        return CoM_parts
            
    def get_center_of_mass(self):
        '''This function returns the center of mass of the full model'''
        
        CoM_parts = self.get_center_of_mass_parts()
        CoM = []
                
        for i in range(3):
            CoM_temp = 0.
            for body in self.bodies:
                CoM_temp = CoM_temp + CoM_parts[body][i]
            CoM.append(CoM_temp/len(self.bodies))
        return CoM
        
    def get_base_of_support(self):
        '''This function returns the location of the base of support'''
        sensors = {}
        BoS = {}
        ground_contacts = self.ground_contacts
        
        for name in ground_contacts:
            if np.abs(ground_contacts[name][2]) > 0:
                BoS[name] = self.ground_contact_positions[name]
            else:
                BoS[name] = [0, 0, 0]
                
        return BoS
                
    def get_desired_bos(self):
        '''This function finds the desired location in the middle of the BoS'''
        #: Calculate base of support
        BoS = self.get_base_of_support()
        #: 2D: last coordinate is the fore-aft direction
        BoS_toe_2Dstance = np.maximum(BoS['TOE_LEFT'][2], BoS['TOE_RIGHT'][2])
        BoS_heel_2Dstance = np.maximum(BoS['HEEL_LEFT'][2], BoS['HEEL_RIGHT'][2])
        
        DES_BoS = (BoS_toe_2Dstance+BoS_heel_2Dstance)/2
        
        return DES_BoS

    def get_center_of_pressure(self, leg = 'both'):
        '''This function finds the center of pressure of the left foot (input 'left'),
        the right foot (input 'right') or both feet (input 'both', default)'''
        
        #2D implementation
        ground_contacts = self.ground_contacts
        if leg == 'left':
            #Calculate for the left leg
            CoP = (ground_contacts['TOE_LEFT'][2]*0.06 - ground_contacts['HEEL_LEFT'][2]*0.06)/(
                ground_contacts['TOE_LEFT'][2] + ground_contacts['HEEL_LEFT'][2])
        elif leg == 'right':
            #Calculate for the right leg
            CoP = (ground_contacts['TOE_RIGHT'][2]*0.06 - ground_contacts['HEEL_RIGHT'][2]*0.06)/(
                ground_contacts['TOE_RIGHT'][2] + ground_contacts['HEEL_RIGHT'][2])
        else:
            #Calculate for both legs
            CoP = (ground_contacts['TOE_RIGHT'][2]*0.06 - ground_contacts['HEEL_RIGHT'][2]*0.06 + 
                ground_contacts['TOE_LEFT'][2]*0.06 - ground_contacts['HEEL_LEFT'][2]*0.06)/(
                ground_contacts['TOE_RIGHT'][2] + ground_contacts['HEEL_RIGHT'][2] + 
                ground_contacts['TOE_LEFT'][2] + ground_contacts['HEEL_LEFT'][2])
        return CoP
            
