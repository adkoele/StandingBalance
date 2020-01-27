from .muscle_joint.muscle_joint_interface import MuscleJointInterface
from .muscle.muscle_system import MuscleSystemGen
from .joint.joint_system import JointSystemGen
from .dae_generator import DaeGenerator
""" MusculoSkeletalSystem class """

import os
import sys

import numpy as np

import farms_pylog as biolog
biolog.set_level('error')


class MusculoSkeletalSystem(object):
    """ Class to generate musculo-skeletal module.
    1. Muscle Generation
    2. Joint Generation
    3. Muscle-Joint Generation : Binding muscles and joints"""

    def __init__(self, config_file=None, opts=None):
        """ Initialize the joints and muscles.
        Need to initialize the class with a valid json file"""
        if config_file is None:
            raise RuntimeError('Missing config file ..... \n\n\n')
        elif not os.path.isfile(config_file):
            raise RuntimeError('Wrong config file ..... \n\n\n')
        else:
            self.config_file = config_file

        #: Attributes
        self.dae = DaeGenerator()
        self.muscle_sys = None
        self.joint_sys = None
        self.state = {'muscles': None, 'joints': None}

        #: Generate the muscles in the system
        self.generate_muscles(opts)
        self.generate_joints()
        self.generate_muscle_joint_interface()

        #: Print the generated system.
        # self.print_system()

    def generate_muscles(self, opts=None):
        """This function creates muscle objects based on the config file.
        The function stores the created muscle objects in a dict."""
        self.muscle_sys = MuscleSystemGen(self.config_file, self.dae)
        #: Setup integrator
        '''for name in self.muscle_sys.muscles.iterkeys():
            dae = self.muscle_sys.muscles[name].dae
            
        if dae != 0:'''
        self.muscle_sys.setup_integrator(integration_method='idas',
                                         opts=opts) #rk
        return self.muscle_sys

    def generate_joints(self):
        """This function creates joint objects based on the config file.
        The function stores the created muscle objects in a dict."""
        try:
            self.joint_sys = JointSystemGen(self.config_file)
        except KeyError:
            return None
        return self.joint_sys

    def generate_muscle_joint_interface(self):
        """This function creates interface between muscles and joint."""
        try:
            MuscleJointInterface(self.config_file,
                                 self.muscle_sys.muscles,
                                 self.joint_sys.joints)
        except AttributeError:
            biolog.warning('No joints present in the system')

    def step(self, joint_positions, dt=0.001):
        """ Step the complete bio-mechanical system.

        Parameters
        ----------
        self: type
            description
        dt: float
            Integration time step
        muscle_stim: dict
            Dictionary of muscle activat
ions
        """
        for name in self.joint_sys.joints.iterkeys():
            self.joint_sys.positions[name] = joint_positions[name]
        self.joint_sys.step(dt)
        self.state['muscles'] = self.muscle_sys.step()
        return

    def print_system(self):
        """
        Print the muscles and joints generated in the musculoskeletalsystem.
        """
        #biolog.info('Muscles created in system : ')
        #for j, muscle in enumerate(self.muscle_sys.muscles.itervalues()):
        #    print('{}. {}'.format(j, muscle.name))

        #biolog.info('Joints created in system : ')
        #for j, joint in enumerate(self.joint_sys.joints.itervalues()):
        #    print('{}. {}'.format(j, joint.name))
        return True

