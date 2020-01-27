import numpy as np
       
class reflex_controller(object):
    ''' This class describes the reflex controller, a general one and the one by Geyer and Herr'''
    def __init__(self, cpg = 0, reflex = 1, geyer = 1,  inhibition = 0):
        ''' geyer = 1 means that Geyer and Herr's model is used.
        cpg = 0 means that there is no cpg controller, 1 means cpg
        inhibition = 0 means that there is no inhibition from antagonist muscles
        '''
        self.inhibition = inhibition
        self.reflex = reflex
        self.geyer_reflex = geyer
        self.cpg = cpg
        self.inhibition = inhibition
        
        self.in_reflex = {}
        self.in_cpg = {}
        self.mn = {}
        self.activations = {}
            
    #Initialization functions
    def initialize_motoneurons(self, body_communication):
        for name in body_communication.muscles:
            self.mn[name] = 0.
    
    #Update functions
    def run_controller(self, body_communication, cparams, cpg = None, net = None):
        if self.reflex == 1:
            self.find_in_reflex(body_communication, cparams)
        else:
            for name in body_communication.muscles:
                self.in_reflex[name] = 0.    
        if self.cpg == 1:
            self.in_cpg = cpg.step_cpg(net, body_communication, cparams)
        else:
            for name in body_communication.muscles:
                self.in_cpg[name] = 0.       
        self.update_motoneurons(body_communication, cparams)
        self.update_activations(body_communication, cparams)
    
    def find_in_reflex(self, body_communication, cparams):
        for name in body_communication.muscles:
            # Find phase
            if '_l' in name:
                if body_communication.DS['L'] or body_communication.stance['L']:
                    phase = 'stance'
                else:
                    phase = 'swing'
            elif '_r' in name:
                if body_communication.DS['R'] or body_communication.stance['R']:
                    phase = 'stance'
                else:
                    phase = 'swing'
                    
            # Load time delays for muscle
            td_to_sc = body_communication.td_to_sc[name]
            
            # Do length feedback
            l_ce = body_communication.delay_states.get_lce(td_to_sc,name)/ body_communication.l_opt[name]
            l_ce_off =  cparams.get('l_off_' +phase+'_'+name)
            in_len = self.interneuron(l_ce, cparams.get('w_len_' +phase+'_'+name), l_ce_off)
            
            # Do force feedback
            f_se = body_communication.delay_states.get_fse(td_to_sc, name)/body_communication.f_max[name]
            in_for = self.interneuron(f_se, cparams.get('w_for_'+phase+'_'+name), 0.)
            
            # Do trunk feedback
            in_trunk = self.interneuron_trunk(body_communication.delay_states.get_trunk_angle(td_to_sc), 
                body_communication.delay_states.get_trunk_angular_velocity(td_to_sc), 
                cparams.get('kp_' +phase+'_'+name), cparams.get('kd_' +phase+'_'+name), cparams.get('qref'))
            
            #Add more interneurons for geyer
            if self.geyer_reflex == 1:
                in_hyper, in_trunk, in_DS, in_len, in_for, in_qto = self.geyer_interneurons(
                    name, in_for, in_len, in_trunk, body_communication, cparams, phase)
            else:
                in_hyper = 0.
                in_DS = 0.
                in_qto = 0.
                
            self.in_reflex[name] = in_len + in_for + in_trunk + in_qto + in_DS + in_hyper
        
    def update_motoneurons(self, body_communication, cparams):
        for name in body_communication.muscles:
            if self.cpg == 1:
                mn_signal = self.motorneuron(self.in_reflex[name], self.in_cpg[name], 
                    cparams.get('w_cpg_flex_'+name) + cparams.get('w_cpg_ext_'+name))
            else:
                mn_signal = self.motorneuron(self.in_reflex[name]) 
            #self.mn[name] = np.maximum(0., mn_signal)
            #if mn_signal < 0:
            #    mn_signal = 0
            self.mn[name] = mn_signal

    def update_activations(self, body_communication, cparams):
        if self.inhibition == 0:
            for name in body_communication.muscles:
                td_from_sc = body_communication.td_from_sc[name]
                if td_from_sc == 0:
                    self.activations[name] = self.activation(self.mn[name], cparams.get('s0_'+name))
                else:
                    self.activations[name] = self.activation(body_communication.delay_states.get_mn
                        (td_from_sc,name), cparams.get('s0_'+name))
        else:
            for name in body_communication.muscles:
                td_from_sc = body_communication.td_from_sc[name]
                if len(body_communication.muscle_inhibitions[name]) == 1:
                    self.activations[name] = self.activation(body_communication.delay_states
                        .get_mn(td_from_sc,name),  body_communication.delay_states.get_mn(td_from_sc,names(0)),
                        cparams.get('w_inh_'+names(0)))
                    
                # there are two muscles on the other side
                elif len(body_communication.muscle_inhibitions[name]) == 2:
                    self.activations[name] = self.activation(body_communication.delay_states
                        .get_mn(td_from_sc,name), body_communication.delay_states.get_mn(td_from_sc,names(0)),
                        cparams.get('w_inh_'+names(0)), body_communication.delay_states.get_mn(td_from_sc,
                        names(1)), cparams.get('w_inh_'+names(1)))
    
    def geyer_interneurons(self, name, in_for, in_len, in_trunk, body_communication, cparams, phase):
        # Load time delays for muscle
        td_to_sc = body_communication.td_to_sc[name]
        if '_r' in name:
            in_trunk = in_trunk*cparams.get('kbw_' +phase+'_'+name) * \
                body_communication.delay_states.get_grf(td_to_sc,'Ry')/body_communication.weight
        else:
            in_trunk = in_trunk*cparams.get('kbw_' +phase+'_'+name) * \
                body_communication.delay_states.get_grf(td_to_sc,'Ly')/body_communication.weight
            
        # Do hyperextension feedback
        in_hyper = 0.
        for mjoint in body_communication.muscle_joints[name]:
            in_hyper += self.hyperextension_interneuron(body_communication.delay_states.get_angle(td_to_sc, mjoint),
                cparams.get('kphi_' +phase +'_'+name), cparams.get('phik_off'+name[-2:]),
                body_communication.delay_states.get_angular_velocity(td_to_sc,mjoint))
               
        # Do double-support feedback
        if '_l' in name:
            in_DS = self.doublesupport_interneuron(cparams.get('kbw_' +phase+'_'+name)*body_communication.DS['L'], 
                cparams.get('DS_'+phase+name[-2:]), body_communication.delay_states.get_grf(5,'Ry')/body_communication.weight)
        else:
            in_DS = self.doublesupport_interneuron(cparams.get('kbw_' +phase+'_'+name)*body_communication.DS['R'],
                cparams.get('DS_'+phase+name[-2:]), body_communication.delay_states.get_grf(5,'Ly')/body_communication.weight)
        
        # Do q_to feedback using the trunk_interneuron and doublesupport_interneuron
        if '_l' in name:
            in_qto = self.interneuron_trunk(body_communication.qto['L'], 0., cparams.get('klean_' +phase+'_'+name), 0.,
                cparams.get('qref'))
        else:
            in_qto = self.interneuron_trunk(body_communication.qto['R'], 0., cparams.get('klean_' +phase+'_'+name), 0.,
                cparams.get('qref'))
            
        # Extras: inhibition TA from SOL , ham, glu and hfl add DS delta S in stance, hfl inhibition from ham in swing
        if name == 'tib_ant_l':
            f_se = body_communication.delay_states.get_fse(td_to_sc, 'soleus_l')/body_communication.f_max['soleus_l']
            in_for -= self.interneuron(f_se, cparams.get('w_for_solta_' +phase+name[-2:]))
        elif name == 'tib_ant_r':
            f_se = body_communication.delay_states.get_fse(td_to_sc, 'soleus_r')/body_communication.f_max['soleus_r']
            in_for -= self.interneuron(f_se, cparams.get('w_for_solta_'  +phase+name[-2:]))
        elif name == 'psoas_l':
            l_ce = body_communication.delay_states.get_lce(td_to_sc,'bifemlh_l')/body_communication.l_opt['bifemlh_l']
            l_ce_off =  cparams.get('l_off_' +phase+ '_bifemlh_l')
            in_len -= self.interneuron(l_ce, cparams.get('w_len_hamhfl_' +phase+name[-2:]), l_ce_off)
            in_DS = body_communication.DS['L']*cparams.get('DS_'+phase+name[-2:])
            if phase == 'stance':
                in_trunk = -np.minimum(0,in_trunk)
        elif name == 'psoas_r':
            l_ce = body_communication.delay_states.get_lce(td_to_sc,'bifemlh_r')/ body_communication.l_opt['bifemlh_r']
            l_ce_off =  cparams.get('l_off_' +phase+ '_bifemlh_r')
            in_len -= self.interneuron(l_ce, cparams.get('w_len_hamhfl_' +phase+name[-2:]), l_ce_off)
            in_DS = body_communication.DS['R']*cparams.get('DS_'+phase+name[-2:])
            if phase == 'stance':
                in_trunk = -np.minimum(0,in_trunk)
        elif name == 'glut_max_l':
            in_DS = -body_communication.DS['L']*cparams.get('DS_'+phase+name[-2:])
            in_trunk = np.maximum(0,in_trunk)
        elif name == 'glut_max_r':
            in_DS = -body_communication.DS['R']*cparams.get('DS_'+phase+name[-2:])
            in_trunk = np.maximum(0,in_trunk)
        elif name == 'bifemlh_l' or name == 'bifemlh_r':
            in_DS = 0.
            in_trunk = np.maximum(0,in_trunk)
            
        return in_hyper, in_trunk, in_DS, in_len, in_for, in_qto
    
    #Other functions
        
    #: Different neurons
    def interneuron(self, x, weight = 1, x_off = 0):
        signal = weight*(x - x_off)
        return np.maximum(0,signal)

    def interneuron_trunk(self, q, dq, kp, kd, qref = 0):
        signal = kp*(q-qref) + kd*dq
        return signal
        
    def hyperextension_interneuron(self, phi, kphi, phik_off, velocity):
        signal = -np.maximum(0., kphi*(-phi+ phik_off)*(velocity< 0))
        return signal
        
    def doublesupport_interneuron(self, kbw,DS, GRF):
        signal = -kbw*DS*GRF
        return signal
        
    #Combine different neurons into one motoneuron signal
    def motorneuron(self, x_in_reflex, x_in_cpg = 0., a_in_cpg = 0): #ADD OPTION OF VARIABLE INPUT NUMBER
        signal = (1-a_in_cpg)*x_in_reflex + a_in_cpg*x_in_cpg
        return signal
        
    #Find activation from motoneuron signals    
    def activation(self, x_mn_ex, x_mn_0, x_mn_in1 = 0., w_mn_in1 = 0., x_mn_in2 = 0., w_mn_in2 = 0.):
        signal = x_mn_0 + x_mn_ex + w_mn_in1*x_mn_in1 + w_mn_in2*x_mn_in2
        return np.maximum(0.,np.minimum(1.,signal))

class balance_controller(object):
    
    def __init__(self, deadzone = True, multipath = False, com = True, timedelay = False):
        self.multipath = multipath
        self.deadzone = deadzone
        self.timedelay = timedelay
        self.mn = {}
        self.activations = {}
        self.w_len = {}
        self.l_off = {}
        self.w_pre = {}
        self.w_vce = {}
        self.w_for = {}
        self.bas_act = {}
        self.com = com
        self.dovce = {}
        if self.timedelay:
            self.td_from_sc = {}
            self.td_to_sc = {}
        
    def controller_params(self, muscles, cparams, opti_params = 0, w_pre = False, w_vce = False):
        self.dovce = w_vce;
        
        for name in muscles:
            self.l_off[name] = cparams['l_off_'+name[:-2]]
            self.bas_act[name] = cparams['s0_'+name[:-2]]
            
            if w_pre:
                self.w_pre[name] = cparams['w_pre_'+name[:-2]]
            else:
                self.w_pre[name] = 0.
            
            if opti_params != 0:
                if 'w_pre_'+name[:-2] in opti_params.iterkeys():
                    self.w_pre[name] = opti_params['w_pre_'+name[:-2]].value
                if 'l_off_'+name[:-2] in opti_params.iterkeys():
                    self.l_off[name] = opti_params['l_off_'+name[:-2]].value
                if name[:-2] in opti_params.iterkeys():
                    self.bas_act[name] = opti_params[name[:-2]].value
                        
            if self.timedelay:
                self.td_from_sc[name] = cparams['td_'+name[:-2]]
                self.td_to_sc[name] = cparams['td_'+name[:-2]]
                if opti_params!=0:
                    if 'td_'+name[:-2] in opti_params.iterkeys():
                        self.td_from_sc[name] = int(np.floor(opti_params['td_'+name[:-2]].value))
                        self.td_to_sc[name] = self.td_from_sc[name]
                        
            if self.multipath:
                self.w_len[name] = {}
                self.w_for[name] = {}

                if w_vce:
                    self.w_vce[name] = {}
                
                for name1 in muscles:
                    if name[-2:] == name1[-2:]:
                        w_len_name = 'w_len_'+name[:-2]+'_'+name1[:-2]
                        w_for_name = 'w_for_'+name[:-2]+'_'+name1[:-2]
                        self.w_len[name][name1] = cparams[w_len_name]
                        self.w_for[name][name1] = cparams[w_for_name]
                        if opti_params != 0:
                            if w_len_name in opti_params:
                                self.w_len[name][name1] = opti_params[w_len_name].value
                            if w_for_name in opti_params:
                                self.w_for[name][name1] = opti_params[w_for_name].value
                                
                        if w_vce:
                            w_vce_name = 'w_vce_'+name[:-2]+'_'+name1[:-2]
                            self.w_vce[name][name1] = cparams[w_vce_name]
                            if opti_params != 0 and w_vce_name in opti_params:
                                self.w_vce[name][name1] = opti_params[w_vce_name].value
                    else:
                        self.w_len[name][name1] = 0.
                        self.w_for[name][name1] = 0.
                        if w_vce:
                            self.w_vce[name][name1] = 0.
            else:
                if w_vce:
                    self.w_vce[name] = cparams['w_vce_'+name[:-2]+'_'+name[:-2]]
                    if opti_params != 0 and 'w_vce_'+name[:-2] in opti_params.iterkeys():
                        self.w_vce[name] = opti_params['w_vce_'+name[:-2]].value
                else:
                    self.w_vce[name] = 0.
                
                self.w_len[name] = cparams['w_len_'+name[:-2]+'_'+name[:-2]]
                self.w_for[name] = cparams['w_for_'+name[:-2]+'_'+name[:-2]]
                if opti_params != 0:
                    if 'w_len_'+name[:-2] in opti_params.iterkeys():
                        self.w_len[name] = opti_params['w_len_'+name[:-2]].value
                    if 'w_for_'+name[:-2] in opti_params.iterkeys():
                        self.w_for[name] = opti_params['w_for_'+name[:-2]].value
            
        if self.deadzone:
            self.toe_tol = cparams['toe_tol']
            self.heel_tol = cparams['heel_tol']
            self.zone = cparams['zone']
            if opti_params != 0:
                if 'toe_tol' in opti_params.iterkeys():
                    self.toe_tol = opti_params['toe_tol'].value
                if 'heel_tol' in opti_params.iterkeys():
                    self.heel_tol = opti_params['heel_tol'].value
                if 'zone' in opti_params.iterkeys():
                    self.zone = opti_params['zone'].value
        else:
            self.toe_tol = 0.07
            self.heel_tol = 0.07
            self.zone = 0.
            if opti_params != 0:
                if 'tol' in opti_params.iterkeys():
                    self.toe_tol = opti_params['tol'].value
                    self.heel_tol = opti_params['tol'].value
           
        '''if not self.com: 
            # make sure that they are not both active at the same time
            self.heel_tol = (1 -self.toe_tol)*self.heel_tol'''
            
        
        #print(self.td_to_sc)
        #print(self.td_from_sc)
        #print(self.zone)
        #print(self.w_for)
        #print(self.w_len)
        #print(self.w_vce)
        #print(self.bas_act)
        #print(self.l_off)

    def balance_controller_fb(self, muscles, webots_communication, body_communication):
        #: Calculate center of pressure
        CoP = webots_communication.get_center_of_pressure()
        #: Calculate center of mass
        CoM = webots_communication.get_center_of_mass()
        
        #: Calculate base of support
        BoS = webots_communication.get_base_of_support()
        DES_BoS = webots_communication.get_desired_bos()
        
        BoS_toe_2Dstance = np.maximum(BoS['TOE_LEFT'][2], BoS['TOE_RIGHT'][2])
        BoS_heel_2Dstance = np.maximum(BoS['HEEL_LEFT'][2], BoS['HEEL_RIGHT'][2])
        
        F_diff = self.interneuron_pressure(body_communication.delay_states.get_contact_force(20))

        if self.com:
            if (BoS_toe_2Dstance - CoM[2]) < self.toe_tol:
                dist_toe = self.toe_tol - (BoS_toe_2Dstance - CoM[2])
                if dist_toe < self.zone*self.toe_tol:
                    factor = dist_toe/(self.zone*self.toe_tol)
                else:
                    factor = 1.
                
            elif (CoM[2] - BoS_heel_2Dstance) < self.heel_tol:
                dist_heel = self.heel_tol - (CoM[2] - BoS_heel_2Dstance)
                if dist_heel < self.zone*self.toe_tol:
                    factor = -dist_heel/(self.zone*self.toe_tol)
                else:
                    factor = -1.
            
            else:
                factor = 0.
                
        else:
            #Based on force
            forces = body_communication.delay_states.get_contact_force_ma(40) #CHeck if time delay number is correct
            toe_force = (forces['TOE_LEFT']+forces['TOE_RIGHT'])/body_communication.weight
            heel_force = (forces['HEEL_LEFT']+forces['HEEL_RIGHT'])/body_communication.weight
            if toe_force > self.toe_tol:
                for_toe = toe_force - self.toe_tol
                if for_toe < self.zone*self.toe_tol:
                    factor = for_toe/(self.zone*self.toe_tol)
                else:
                    factor = 1.
            elif heel_force > self.heel_tol:
                for_heel = heel_force - self.heel_tol
                if for_heel < self.zone*self.heel_tol:
                    factor = -for_heel/(self.zone*self.heel_tol)
                else:
                    factor = -1.
            else:
                factor = 0.

        for name in muscles:
            # Load time delays for muscle
            if self.timedelay:
                td_to_sc = self.td_to_sc[name]
                td_from_sc = self.td_from_sc[name]
            else:
                td_to_sc = body_communication.td_to_sc[name]
                td_from_sc = body_communication.td_from_sc[name]

            basal_activation = self.bas_act[name]
            

            # Do pressure feedback
            in_pre = factor*self.w_pre[name]*F_diff
            
            # Multifeedback
            if self.multipath:
                in_len = 0.
                in_for = 0.
                in_vce = 0.
					
                for name1 in muscles:
                    if self.timedelay:
                        td_to_sc = self.td_to_sc[name1]
                    else:
                        td_to_sc = body_communication.td_to_sc[name1]
                    
                    # Do length feedback
                    l_ce = body_communication.delay_states.get_lce(td_to_sc,name1)/ body_communication.l_opt[name1]
                    l_ce_off = self.l_off[name1]
                    in_len += self.interneuron(l_ce, factor*self.w_len[name][name1], l_ce_off)
                    
                    # Do velocity feedback
                    if self.dovce:
						v_ce = body_communication.delay_states.get_vce(td_to_sc,name1)/ body_communication.l_opt[name1]
						in_vce += self.interneuron(v_ce, factor*self.w_vce[name][name1])
                    
                    #Do muscle force feedback
                    f_se = body_communication.delay_states.get_fse(td_to_sc,name1)/ body_communication.f_max[name1]
                    in_for += self.interneuron(f_se, factor*self.w_for[name][name1])
            else:
                # Do length feedback
                l_ce = body_communication.delay_states.get_lce(td_to_sc,name)/ body_communication.l_opt[name]
                l_ce_off = self.l_off[name]
                in_len = self.interneuron(l_ce, factor*self.w_len[name], l_ce_off)
                
                # Do velocity feedback
                v_ce = body_communication.delay_states.get_vce(td_to_sc,name)/ body_communication.l_opt[name]
                in_vce = self.interneuron(v_ce, factor*self.w_vce[name])
                
                #Do muscle force feedback
                f_se = body_communication.delay_states.get_fse(td_to_sc,name)/ body_communication.f_max[name]
                in_for = self.interneuron(f_se, factor*self.w_for[name])
                
            # Motoneuron
            if self.w_pre[name] != 0 and self.w_vce != 0:
                print("w_pre: Check this")
                a_len = 1./4
                a_for = 1./4
                a_pre = 1./4
                a_vce = 1./4
            elif self.w_pre[name] != 0:
                a_len = 1./3
                a_for = 1./3
                a_pre = 1./3
                a_vce = 0.
            elif self.dovce:
                a_len = 1./3
                a_for = 1./3
                a_vce = 1./3
                a_pre = 0.
            else:
                a_len = 1./2
                a_for = 1./2
                a_vce = 0.
                a_pre = 0
            self.mn[name] =  a_len*in_len + a_for * in_for + a_pre * in_pre + a_vce * in_vce
            
            #this should be passed on somewhere else
            self.activations [name] = np.maximum(0., np.minimum(
                basal_activation + body_communication.delay_states.get_mn(td_from_sc,name),1.))
                
            #print("Name: {}, Factor: {}, Bas: {}, Act:{}".format(name, factor, basal_activation, self.activations[name]))
            
    def interneuron(self, x, weight = 1., x_off = 0.):
        signal = weight*np.maximum(0.,(x - x_off))
        return signal
        
    def interneuron_pressure(self,forces):
        F1 = forces['HEEL_RIGHT']+forces['HEEL_LEFT']
        F2 = forces['TOE_RIGHT']+forces['TOE_LEFT']
        
        if (F1+F2) < 1e-3:
            F_diff = 0
        else:
            F_diff = (F1-F2)/(F1+F2)
        if np.isnan(F_diff):
            F_diff = 0.
        return F_diff

'''class initialization_controller(object):
    
    def __init__(self,):
        # initialization of gait
        
        self.phase = 0
        self.mn = {}
        self.activations = {}
        
    def controller(self):
        if self.phase == 0: #move center of mass forward
            
        if self.phase == 1: #activate soleus/gastroc in DS leg
            
        if self.phase == 2: #swing phase
    

'''
