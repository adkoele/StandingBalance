import os

import biolog
import yaml

#from body_communication import body_communication

class parameters(object):
    ''' This class handles the parameters of the controller, like feedback gains
    and basal activation etc. '''
    def __init__(self):
        self.cparams = {}
        self.symmetry = 0
        
    def initialize_parameters(self, control_path, body_communication, control_name = 'geyer', symmetry = 0, multipath = False):
        self.symmetry = symmetry      
        self.control_name = control_name  
        self.update_parameters(control_path, body_communication, multipath)
      
    def update_parameters(self, control_path, body_communication, multipath = False):
        self.read_yaml_file(control_path)
        self.get_parameter_names()
        self.check_parameter_names(body_communication, multipath)
    
    #: Read the yaml file
    def read_yaml_file(self, control_path):
        try:
            stream = file(os.path.realpath(control_path), 'r')
            self.cparams = yaml.load(stream)
            '''biolog.info('Successfully loaded the file : {}'.format(
                os.path.split(control_path)[-1]))'''
            return
        except ValueError:
            biolog.error('Unable to read the file {}'.format(control_path))
            raise ValueError()
            
    #: This function returns the parameters that should be included depending on the controller
    def get_parameter_names(self):
        #: Based on name of controller, define the parameters that exist
        self.muscle_params_mult = []
        if self.control_name == 'geyer':
            self.muscle_params = ['s0', 'w_cpg_ext', 'w_cpg_flex', 'w_len_stance', 'w_for_stance', 'l_off_stance', 'kp_stance', 'kd_stance' , 'kphi_stance', 'kbw_stance', 'klean_stance',
                'w_len_swing', 'w_for_swing', 'l_off_swing', 'kp_swing', 'kd_swing' , 'kphi_swing', 'kbw_swing', 'klean_swing']
            self.extra_params = ['qref', 'phik_off_l', 'DS_stance_l', 'DS_swing_l', 'w_len_hamhfl_stance_l', 
                    'w_for_solta_stance_l', 'w_len_hamhfl_swing_l', 'w_for_solta_swing_l',
                    'phik_off_r', 'DS_stance_r', 'DS_swing_r', 'w_len_hamhfl_stance_r', 
                    'w_for_solta_stance_r', 'w_len_hamhfl_swing_r', 'w_for_solta_swing_r']
        elif self.control_name == 'geyer-cpg':
            self.muscle_params = ['s0', 'w_cpg_ext', 'w_cpg_flex', 'w_len_stance', 'w_for_stance', 'l_off_stance', 'kp_stance', 'kd_stance' , 'kphi_stance', 'kbw_stance', 'klean_stance',
                'w_len_swing', 'w_for_swing', 'l_off_swing', 'kp_swing', 'kd_swing' , 'kphi_swing', 'kbw_swing', 'klean_swing']
            if self.symmetry == 0:
                self.extra_params = ['qref', 'phik_off_l', 'DS_stance_l', 'DS_swing_l', 'w_len_hamhfl_stance_l', 
                    'w_for_solta_stance_l', 'w_len_hamhfl_swing_l', 'w_for_solta_swing_l',
                    'phik_off_r', 'DS_stance_r', 'DS_swing_r', 'w_len_hamhfl_stance_r', 
                    'w_for_solta_stance_r', 'w_len_hamhfl_swing_r', 'w_for_solta_swing_r']
            else:
                self.extra_params = ['qref', 'phik_off_l', 'DS_stance_l', 'DS_swing_l', 'w_len_hamhfl_stance_l', 
                    'w_for_solta_stance_l', 'w_len_hamhfl_swing_l', 'w_for_solta_swing_l',
                    'phik_off_r', 'DS_stance_r', 'DS_swing_r', 'w_len_hamhfl_stance_r', 
                    'w_for_solta_stance_r', 'w_len_hamhfl_swing_r', 'w_for_solta_swing_r']
        
        elif self.control_name == 'BIOROB':
            self.muscle_params = ['s0', 'w_cpg_ext', 'w_cpg_flex', 'w_len_stance', 'w_for_stance', 'l_off_stance', 'kp_stance', 'kd_stance',
                'w_len_swing', 'w_for_swing', 'l_off_swing', 'kp_swing', 'kd_swing']
            self.extra_params = {'qref'}
        elif self.control_name == 'balance':
            self.muscle_params = ['s0', 'l_off']
            self.muscle_params_mult = ['w_for', 'w_len', 'w_vce']
            self.extra_params = {'toe_tol', 'heel_tol', 'zone'}
        elif self.control_name == 'balance_time':
            self.muscle_params = ['s0', 'l_off', 'td']
            self.muscle_params_mult = ['w_for', 'w_len', 'w_vce']
            self.extra_params = {'toe_tol', 'heel_tol', 'zone'}
            
    #: This function adds zeros for the parameters that were not initialized
    def check_parameter_names(self, body_communication, multipath = False):
        #: Check if all params are present, add to zero otherwise
        for muscle in body_communication.muscles:
            for param in self.muscle_params:
                if self.symmetry == 0:
                    if param+'_'+muscle not in self.cparams: #type(self.cparams[param+'_'+muscle]) is None:
                        self.cparams[param+'_'+muscle] = 0.
                        biolog.info('Parameter: {} was not added, using 0'.format([param+'_'+muscle]))
                elif 'geyer' in self.control_name or 'BIOROB' in self.control_name:
                    if param+'_'+muscle not in self.cparams: #type(self.cparams[param+'_'+muscle]) is None:
                        self.cparams[param+'_'+muscle] = 0.
                        biolog.info('Parameter: {} was not added, using 0'.format([param+'_'+muscle]))
                else:
                    if param+'_'+muscle[:-2] not in self.cparams: #type(self.cparams[param+'_'+muscle[:-2]]) is None:
                        self.cparams[param+'_'+muscle[:-2]] = 0.
                        biolog.info('Parameter: {} was not added, using 0'.format([param+'_'+muscle[:-2]]))
           
        for param in self.muscle_params_mult:
            for muscle in body_communication.muscles:
                for muscle2 in body_communication.muscles:
                    if self.symmetry == 0:
                        if param+'_'+muscle+'_'+muscle2 not in self.cparams: #type(self.cparams[param+'_'+muscle]) is None:
                            if muscle == muscle2 and param+'_'+muscle in self.cparams:
                                self.cparams[param+'_'+muscle+'_'+muscle2] = self.cparams[param+'_'+muscle]
                            else:                              
                                self.cparams[param+'_'+muscle+'_'+muscle2] = 0.
                                biolog.info('Parameter: {} was not added, using 0'.format([param+'_'+muscle+'_'+muscle2]))
                    else:
                        if param+'_'+muscle[:-2]+'_'+muscle2[:-2] not in self.cparams: #type(self.cparams[param+'_'+muscle[:-2]]) is None:
                            if muscle == muscle2 and param+'_'+muscle[:-2] in self.cparams:
                                self.cparams[param+'_'+muscle[:-2]+'_'+muscle2[:-2]] = self.cparams[param+'_'+muscle[:-2]]
                            else:                              
                                self.cparams[param+'_'+muscle[:-2]+'_'+muscle2[:-2]] = 0.
                                biolog.info('Parameter: {} was not added, using 0'.format([param+'_'+muscle[:-2]+'_'+muscle2[:-2]]))
            
        for param in self.extra_params:
            if param not in self.cparams:
                self.cparams[param] = 0.
                biolog.info('Parameter: {} was not added, using 0'.format([param]))
                
    #: This function adds the optimized parameters
    def add_optimization_parameters(self, opti):
        if 'geyer' in self.control_name:
            self.set_geyer_params(opti) 
        else:
            ''' This option is probably better:
            for param_name in self.muscle_params:
                for muscle in body_communication.muscles:
                    if param_name+'_'+muscle in opti.parameters.iterkeys():
                        self.cparams[param_name+'_'+muscle] = opti.parameters[param_name+'_'+muscle].value
            for param_name in self.extra_params:
                if param_name in opti.parameters.iterkeys():
                    self.cparams[param_name] = opti.parameters[param_name].value '''
            for param_name in opti.parameters.iterkeys():
                if self.symmetry == 1:
                    if param_name in self.extra_params:
                        self.cparams[param_name] = opti.parameters[param_name].value     
                    else:
                        self.cparams[param_name+'_l'] = opti.parameters[param_name].value     
                        self.cparams[param_name+'_r'] = opti.parameters[param_name].value     
                else:
                    self.cparams[param_name] = opti.parameters[param_name].value     
    
    def set_geyer_params(self, opti):
        for param_name in opti.parameters.iterkeys():
            if self.symmetry == 1:
                if param_name[0] == 'k':
                    if param_name == 'kp_stance':
                        self.cparams['kp_stance_glut_max_r'] = 0.68*opti.parameters[param_name].value
                        self.cparams['kp_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kp_stance_bifemlh_r'] = opti.parameters[param_name].value
                        self.cparams['kp_stance_glut_max_l'] = 0.68*opti.parameters[param_name].value
                        self.cparams['kp_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kp_stance_bifemlh_l'] = opti.parameters[param_name].value
                    elif param_name == 'kd_stance':
                        self.cparams['kd_stance_glut_max_r'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_bifemlh_r'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_glut_max_l'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_bifemlh_l'] = opti.parameters[param_name].value
                    elif param_name == 'kbw_stance':
                        self.cparams['kbw_stance_glut_max_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_bifemlh_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_glut_max_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_bifemlh_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_vas_int_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_vas_int_l'] = opti.parameters[param_name].value
                    elif param_name == 'klean_swing':
                        self.cparams['klean_swing_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['klean_swing_psoas_l'] = opti.parameters[param_name].value
                    else:
                        self.cparams[param_name+'_l'] = opti.parameters[param_name].value
                        self.cparams[param_name+'_r'] = opti.parameters[param_name].value
                elif param_name == 'qref':
                    self.cparams[param_name] = opti.parameters[param_name].value
                else:
                    self.cparams[param_name+'_l'] = opti.parameters[param_name].value
                    self.cparams[param_name+'_r'] = opti.parameters[param_name].value
                    if param_name == 'l_off_stance_tib_ant':
                        self.cparams['l_off_swing_tib_ant_l'] = opti.parameters['l_off_stance_tib_ant'].value
                        self.cparams['l_off_swing_tib_ant_r'] = opti.parameters['l_off_stance_tib_ant'].value
                    elif param_name == 'w_len_stance_tib_ant':
                        self.cparams['w_len_swing_tib_ant_l'] = opti.parameters["w_len_stance_tib_ant"].value
                        self.cparams['w_len_swing_tib_ant_r'] = opti.parameters["w_len_stance_tib_ant"].value
            else:
                if param_name[0] == 'k':
                    if param_name == 'kp_stance_r':
                        self.cparams['kp_stance_glut_max_r'] = 0.68*opti.parameters[param_name].value
                        self.cparams['kp_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kp_stance_bifemlh_r'] = opti.parameters[param_name].value
                    if param_name == 'kp_stance_l':    
                        self.cparams['kp_stance_glut_max_l'] = 0.68*opti.parameters[param_name].value
                        self.cparams['kp_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kp_stance_bifemlh_l'] = opti.parameters[param_name].value
                    elif param_name == 'kd_stance_r':
                        self.cparams['kd_stance_glut_max_r'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_bifemlh_r'] = opti.parameters[param_name].value
                    elif param_name == 'kd_stance_l':    
                        self.cparams['kd_stance_glut_max_l'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kd_stance_bifemlh_l'] = opti.parameters[param_name].value
                    elif param_name == 'kbw_stance_r':
                        self.cparams['kbw_stance_glut_max_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_psoas_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_bifemlh_r'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_vas_int_r'] = opti.parameters[param_name].value
                    elif param_name == 'kbw_stance_l':    
                        self.cparams['kbw_stance_glut_max_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_psoas_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_bifemlh_l'] = opti.parameters[param_name].value
                        self.cparams['kbw_stance_vas_int_l'] = opti.parameters[param_name].value
                    elif param_name == 'klean_swing_r':
                        self.cparams['klean_swing_psoas_r'] = opti.parameters[param_name].value
                    elif param_name == 'klean_swing_l':    
                        self.cparams['klean_swing_psoas_l'] = opti.parameters[param_name].value
                    else:
                        self.cparams[param_name] = opti.parameters[param_name].value
                else:
                    self.cparams[param_name] = opti.parameters[param_name].value
                    if 'l_off_stance_tib_ant' in param_name:
                        self.cparams['l_off_swing_tib_ant_l'] = opti.parameters['l_off_stance_tib_ant_l'].value
                        self.cparams['l_off_swing_tib_ant_r'] = opti.parameters['l_off_stance_tib_ant_r'].value
                    elif 'w_len_stance_tib_ant' in param_name:
                        self.cparams['w_len_swing_tib_ant_l'] = opti.parameters["w_len_stance_tib_ant_l"].value
                        self.cparams['w_len_swing_tib_ant_r'] = opti.parameters["w_len_stance_tib_ant_r"].value

