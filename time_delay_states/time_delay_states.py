import logging
import numpy as np
from collections import OrderedDict
#from webots_communication import webots_communication
#from body_communication import body_communication

class time_delay_states():
    """Store time delayed variables
    """

    def __init__(self, body_communication, webots_communication, largest_time_delay = 100):
        self.largest_time_delay = largest_time_delay
        self.trunk_angle = np.zeros(largest_time_delay)
        self.trunk_angular_vel = np.zeros(largest_time_delay)
        self.angles = OrderedDict()
        self.angular_velocity = OrderedDict()
        self.f_se = OrderedDict()
        self.l_ce = OrderedDict()
        self.v_ce = OrderedDict()
        self.ground_reaction_force = OrderedDict()
        self.contact_force = OrderedDict()
        self.mn = OrderedDict()

        # Create matrices for joints
        for name in body_communication.joint_positions:
            self.angles[name] = np.zeros(largest_time_delay)
            self.angular_velocity[name] = np.zeros(largest_time_delay)
            
        # Create matrices for muscles
        for name in body_communication.l_ce:
            self.f_se[name] = np.zeros(largest_time_delay)
            self.l_ce[name] = np.zeros(largest_time_delay)
            self.v_ce[name] = np.zeros(largest_time_delay)
            self.mn[name] = np.zeros(largest_time_delay)
    
        for name in webots_communication.ground_reaction_force: 
            self.ground_reaction_force[name] = np.zeros(largest_time_delay)
        
        for name in webots_communication.ground_contacts:   
            self.contact_force[name] = np.zeros(largest_time_delay)
    
    def step(self, body_communication, webots_communication, mn):
                
        #determine trunk angle
        trunk = body_communication.joint_positions['trunk']
        new_trunk_angle = np.zeros(self.largest_time_delay)
        new_trunk_angle[1:] = self.trunk_angle[:-1]
        new_trunk_angle[0] = trunk #np.arccos(trunk[4])
        self.trunk_angle = new_trunk_angle
        
        new_trunk_angular_vel = np.zeros(self.largest_time_delay)
        new_trunk_angular_vel[1:] = self.trunk_angular_vel[:-1]
        new_trunk_angular_vel[0] = (self.trunk_angle[0]-self.trunk_angle[1])*1000.
        self.trunk_angular_vel = new_trunk_angular_vel

        # Create matrices for joints
        for name in body_communication.joint_positions:
            # angle
            cur_state = body_communication.joint_positions[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.angles[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.angles[name] = new_states
                
            # angular velocity
            cur_state = body_communication.joint_velocities[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.angles[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.angular_velocity[name] = new_states
            
        # Create matrices for muscles
        for name in body_communication.l_ce:
            #f_se
            cur_state = body_communication.f_se[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.f_se[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.f_se[name] = new_states

            #l_ce
            cur_state = body_communication.l_ce[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.l_ce[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.l_ce[name] = new_states
            
            #v_ce
            cur_state = body_communication.v_ce[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.v_ce[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.v_ce[name] = new_states
            
            #mn
            cur_state = mn[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.mn[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state
            self.mn[name] = new_states
        
        for name in webots_communication.ground_reaction_force: 
            cur_state = webots_communication.ground_reaction_force[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.ground_reaction_force[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = cur_state 
            self.ground_reaction_force[name] = new_states
            
        for name in webots_communication.ground_contacts:   
            cur_state = webots_communication.ground_contacts[name]
            new_states = np.zeros(self.largest_time_delay)
            previous_states = self.contact_force[name]

            new_states[1:] = previous_states[:-1]
            new_states[0] = -cur_state[2]
            self.contact_force[name] = new_states

    def get_angle(self, time_delay, name):
        cur_angle = self.angles[name]
        return cur_angle[time_delay-1]
        
    def get_trunk_angle(self, time_delay):
        cur_angle = self.trunk_angle
        return cur_angle[time_delay-1]
        
    # Get all ground reaction forces
    def get_ground_reaction_force(self, time_delay):
        cur_force = {}
        for name in self.ground_reaction_force.iterkeys():
            cur_force[name] = self.ground_reaction_force[name][time_delay-1]
        return cur_force
        
    #Get ground reaction force on left or right side
    def get_grf(self, time_delay, name):
        print(self.ground_reaction_force)
        cur_grf = self.ground_reaction_force[name]
        return cur_grf[time_delay-1]
        
    # Get force at contact points
    def get_contact_force(self, time_delay):
        cur_force = {}
        for name in self.contact_force.iterkeys():
            cur_force[name] = self.contact_force[name][time_delay-1]
        return cur_force
        
    # Get moving average force at contact points
    def get_contact_force_ma(self, time_delay):
        cur_force = {}
        for name in self.contact_force.iterkeys():
            cur_force[name] = np.mean(self.contact_force[name][time_delay-11:time_delay-1])
        return cur_force
        
    def get_trunk_angular_velocity(self, time_delay):
        cur_angular_vel = self.trunk_angular_vel
        return cur_angular_vel[time_delay-1]
        
    def get_angular_velocity(self, time_delay, name):
        cur_angular_vel = self.angular_velocity[name]
        return cur_angular_vel[time_delay-1]
        
    def get_lce(self, time_delay, name):
        cur_lce = self.l_ce[name]
        return cur_lce[time_delay-1]
        
    def get_vce(self, time_delay, name):
        cur_vce = self.v_ce[name]
        return cur_vce[time_delay-1]
        
    def get_mn(self, time_delay, name):
        cur_mn = self.mn[name]
        return cur_mn[time_delay-1]
        
    def get_fse(self, time_delay, name):
        cur_fse = self.f_se[name]
        return cur_fse[time_delay-1]

    
        
        
