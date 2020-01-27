from ..musculo_skeletal_parameters import MuscleJointParameters
from .muscle_joint import GeyerMuscleJoint
"""Establish the interface between muscle and joints"""

import os
import sys
from collections import OrderedDict
import yaml
import farms_pylog as biolog
biolog.set_level('error')


class MuscleJointInterface(object):
    """Generate the interface between muscle and joints
    """

    def __init__(self, muscle_joint_config_path, muscles, joints):
        """Initialize.

        Parameters
        ----------
        muscle_joint_config : <str>
            Path to muscle joint system configuration path.
        muscles: <dict>
            Dictionary containing list of muscles
        joints: <dict>
            Dictionary containing list of joints
        """
        super(MuscleJointInterface, self).__init__()
        self.muscle_joint_config_path = muscle_joint_config_path
        self.muscle_joint_config = None  #: Muscle Configuration data

        #: Methods
        self.load_config_file()
        self.generate_muscle_joint_interface(muscles, joints)

    def load_config_file(self):
        """Load the animal configuration file"""
        try:
            stream = open(
                os.path.realpath(self.muscle_joint_config_path), 'r')
            self.muscle_joint_config = yaml.load(stream)
            biolog.info('Successfully loaded the file : {}'.format(
                os.path.split(self.muscle_joint_config_path)[-1]))
            return
        except ValueError:
            biolog.error('Unable to read the file {}'.format(
                self.muscle_joint_config_path))
            raise ValueError()

    def generate_muscle_joint_parameters(self, muscle_name):
        """To create muscle-joint parameters from json file.

        Parameters
        ----------
        muscle_name: string
            Name of the muscle
        """

        param = self.muscle_joint_config['muscles'][muscle_name]
        if param is None:
            print('Invalid muscle name')
            sys.exit(1)

        if param['muscle_type'] == 'mono':
            return [MuscleJointParameters(
                muscle_type=param['muscle_type'],
                r_0=param['r_0'],
                joint_attach=param['joint_attach'],
                theta_max=param['theta_max'],
                theta_ref=param['theta_ref'],
                direction=param['direction'])]

        elif param['muscle_type'] == 'bi':
            return [
                MuscleJointParameters(
                    muscle_type=param['muscle_type'],
                    r_0=param['r_0'],
                    joint_attach=param['joint_attach'],
                    theta_max=param['theta_max'],
                    theta_ref=param['theta_ref'],
                    direction=param['direction']
                ),
                MuscleJointParameters(
                    muscle_type=param['muscle_type'],
                    r_0=param['r_02'],
                    joint_attach=param['joint_attach2'],
                    theta_max=param['theta_max2'],
                    theta_ref=param['theta_ref2'],
                    direction=param['direction2']
                )
            ]

    def generate_muscle_joint_interface(self, muscles, joints):
        """ Creates the muscle joint interface."""
        for muscle_name in list(muscles.keys()):
            muscle_joint_param = self.generate_muscle_joint_parameters(
                muscle_name)
            # Get the muscle
            muscle = muscles[muscle_name]
            for mj in muscle_joint_param:
                joint = joints[mj.joint_attach]
                muscle.muscle_joints.append(
                    GeyerMuscleJoint(muscle, joint, mj))
                muscle.initialize_muscle_length()
        return
