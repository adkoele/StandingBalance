from ..musculo_skeletal_parameters import JointParameters
from .joint import GeyerJoint
"""Generation of Multiple Joint Models in the System."""
import os
from collections import OrderedDict
import yaml
import farms_pylog as biolog
biolog.set_level('error')


class JointSystemGen(object):
    """Generate Joint Models for the the animal.
    """

    def __init__(self, joint_config_path):
        """Initialize.

        Parameters
        ----------
        joint_config : <str>
            Path to joint system configuration path.
        """
        super(JointSystemGen, self).__init__()
        self.joint_config_path = joint_config_path
        self.joint_config = None  #: Muscle Configuration data

        #: Attributes
        self.joints = OrderedDict()  #: List of joints
        self.positions = {}  # : Dict to store the joints
        self.prev_positions = {}  # : Dict to store the joints
        self.velocities = {}

        #: Methods
        self.load_config_file()
        self.generate_joints()
        self.generate_joint_position_container()

    def load_config_file(self):
        """Load the joint configuration file"""
        try:
            stream = open(
                os.path.realpath(self.joint_config_path), 'r')
            self.joint_config = yaml.load(stream)
            #biolog.info('Successfully loaded the file : {}'.format(
            #    os.path.split(self.joint_config_path)[-1]))
            return
        except ValueError:
            biolog.error('Unable to read the file {}'.format(
                self.joint_config_path))
            raise ValueError()

    def generate_joints(self):
        """ Generate muscles. """
        for _, joint in sorted(self.joint_config['joints'].items()):
            if joint['model'].lower() == 'geyer':
                self.joints[joint['name']] = GeyerJoint(
                    JointParameters(**joint))
            #biolog.debug('Created joint {}'.format(
            #    self.joints[joint['name']].name))
        return

    def generate_joint_position_container(self):
        """Generate joint position container. """
        for name in list(self.joints.keys()):
            self.positions[name] = 0.0

    def step(self, dt):
        """Step joint system."""
        for name, joint in self.joints.items():
            joint.update_angle(self.positions[name])
            joint.step(dt)
        return
