import logging
import numpy as np
import os

class DataLogger():
    """Generate Muscle Models for the the animal.
    """

    def __init__(self, file_path,body_communication,jointopts = ["angles", "velocities", "moments"], muscleopts
        = ["act", "stim", "l_ce", "f_se", "v_ce", "f_ce"], grfopts =  ["LGRF", "RGRF"], grf1opts = None):

        # Save path
        if file_path[-1] != '/':
            file_path = file_path + '/'
        self.file_path = file_path
        # Define what should be logged
        self.joint_log = jointopts
        self.muscle_log = muscleopts
        self.grf_log = grfopts
        
        self.headers = {}
        self.headers['joint'] = ''
        for name in body_communication.joints:
            self.headers['joint'] += str(body_communication.joints[name])
            self.headers['joint'] += " "
        self.headers['muscle'] = ''
        for name in body_communication.muscles:
            self.headers['muscle'] += str(body_communication.muscles[name])
            self.headers['muscle'] += " "
        self.headers['GRF'] = "GRFx GRFy"
        self.headers['trunk'] = "trunk"
        self.headers['touchSensor'] = "HEEL_LEFT TOE_LEFT HEEL_RIGHT TOE_RIGHT"
        self.headers['statemachine'] = "L R"

        self.angles = []
        self.trunk = []
        self.velocities = []
        self.moments = []
        self.act = []
        self.stim = []
        self.f_se = []
        self.l_ce = []
        self.f_ce = []
        self.v_ce = []
        self.l_mtc = []
        self.delta_length = []
        self.LGRF = []
        self.RGRF = []
        self.gc = []
        self.DS = []
        self.stance = []
        self.swing = []
        self.counter = 0

    def write_to_files(self, system):
        np.savetxt(self.file_path+"angles.txt", self.angles, header=self.headers['joint'], comments='')
        np.savetxt(self.file_path+"velocities.txt", self.velocities, header=self.headers['joint'], comments='')
        np.savetxt(self.file_path+"moments.txt", self.moments, header=self.headers['joint'], comments='')
        
        np.savetxt(self.file_path+"act.txt", self.act, header=self.headers['muscle'], comments='')
        np.savetxt(self.file_path+"stim.txt", self.stim, header=self.headers['muscle'], comments='')
        np.savetxt(self.file_path+"l_ce.txt", self.l_ce, header=self.headers['muscle'], comments='')
        np.savetxt(self.file_path+"v_ce.txt", self.v_ce, header=self.headers['muscle'], comments='')
        np.savetxt(self.file_path+"f_ce.txt", self.f_ce, header=self.headers['muscle'], comments='')
        np.savetxt(self.file_path+"f_se.txt", self.f_se, header=self.headers['muscle'], comments='')
        
        np.savetxt(self.file_path+"LGRF.txt", self.LGRF, header=self.headers['GRF'], comments='')
        np.savetxt(self.file_path+"RGRF.txt", self.RGRF, header=self.headers['GRF'], comments='')
                
        np.savetxt(self.file_path+"touchSensor.txt", self.gc, header=self.headers['touchSensor'], comments='')
        np.savetxt(self.file_path+"trunk.txt", self.trunk, header=self.headers['trunk'], comments='')
        
        np.savetxt(self.file_path+"DS.txt", self.DS, header=self.headers['statemachine'], comments='')
        np.savetxt(self.file_path+"stance.txt", self.stance, header=self.headers['statemachine'], comments='')
        np.savetxt(self.file_path+"swing.txt", self.swing, header=self.headers['statemachine'], comments='')
    
    def step(self, body_communication, ground_reaction_force = None, ground_contact = None, state_machine = False):
        # make row that should be appeded
        rowangles = []
        #rowtrunk = []
        rowvelocities = []
        rowmoments = []
        for name in body_communication.joints:
            rowangles.append(body_communication.joint_positions[name])
            rowvelocities.append(body_communication.joint_velocities[name])
            rowmoments.append(body_communication.new_torque[name])
        
        #rowtrunk.append(body_communication.joint_positions['trunk'])
            
        self.angles.append(rowangles)
        #self.trunk.append(rowtrunk)
        self.velocities.append(rowvelocities)
        self.moments.append(rowmoments)

        rowact = []
        rowstim = []
        rowf_se = []
        rowl_ce = []
        rowf_ce = []
        rowv_ce = []
        rowl_mtc = []
        rowdelta_length = []

        for name in body_communication.muscles:
            rowact.append(body_communication.act[name])
            rowstim.append(body_communication.stim[name])
            rowf_se.append(body_communication.f_se[name])
            rowl_ce.append(body_communication.l_ce[name]/body_communication.l_opt[name])
            rowf_ce.append(body_communication.f_ce[name])
            rowv_ce.append(body_communication.v_ce[name])

        self.act.append(rowact)
        self.stim.append(rowstim)
        self.f_se.append(rowf_se)
        self.l_ce.append(rowl_ce)
        self.f_ce.append(rowf_ce)
        self.v_ce.append(rowv_ce)
        
        if ground_reaction_force != None:
            rowlgrf = []
            rowlgrf.append(ground_reaction_force['Lx'])
            rowlgrf.append(ground_reaction_force['Ly'])
            self.LGRF.append(rowlgrf)

            rowrgrf = []
            rowrgrf.append(ground_reaction_force['Rx'])
            rowrgrf.append(ground_reaction_force['Ry'])
            self.RGRF.append(rowrgrf)
            
        if ground_contact != None:
            rowgc = []
            rowgc.append(np.abs(ground_contact['HEEL_LEFT'][2]))
            rowgc.append(np.abs(ground_contact['TOE_LEFT'][2]))
            rowgc.append(np.abs(ground_contact['HEEL_RIGHT'][2]))
            rowgc.append(np.abs(ground_contact['TOE_RIGHT'][2]))
            self.gc.append(rowgc)
        
        if state_machine:    
            rowswing = []
            rowstance = []
            rowDS = []
            rowswing.append(body_communication.swing['L'])
            rowswing.append(body_communication.swing['R'])
            rowstance.append(body_communication.stance['L'])
            rowstance.append(body_communication.stance['R'])
            rowDS.append(body_communication.DS['L'])
            rowDS.append(body_communication.DS['R'])
            self.swing.append(rowswing)
            self.stance.append(rowstance)
            self.DS.append(rowDS)
     
    def add_to_logger(self, body_communication, ground_reaction_force = None, ground_contacts = None, state_machine = False, time_step = 10):
        self.counter += 1
        if self.counter >= time_step:
            self.step(body_communication, ground_reaction_force, ground_contacts, state_machine)
            self.counter = 0
            
