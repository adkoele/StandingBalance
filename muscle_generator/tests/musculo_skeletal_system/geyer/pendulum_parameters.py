#! /usr/bin/env python

"""Test pendulum parameters"""

import numpy as np
import biolog


class SystemParameters(object):
    """Parent class providing main attributes for other sub system
    parameters.

    """

    def __init__(self, name='System'):
        super(SystemParameters, self).__init__()
        self. name = name

    def showParameters(self):
        raise NotImplementedError()

    def msg(self, parameters, units, endl="\n" + 4 * " "):
        """ Message """
        to_print = ("{} parameters : ".format(self.name)) + endl
        for param in parameters:
            to_print += ("{} : {} [{}]".format(param,
                                               parameters[param],
                                               units[param])) + endl
        return to_print


class PendulumParameters(SystemParameters):
    """ Pendulum parameters

    with:
    ----
        Pendulum Parameters:
            - g: Gravity constant [m/s**2]
            - L: Length [m]
            - sin: Sine function
            - mass: mass of the pendulum [kg]
            - Inertia: intertia of the pendulum [kg-m**2]
            - theta_max: Maximum pendulum angle
            - theta_min: Minimum pendulum angle
            - PERTURBATION: Boolean to apply perturbation

    Examples:

        >>> pendulum_parameters = PendulumParameters(g=9.81, L=0.1)

    Note that not giving arguments to instanciate the object will result in the
    following default values:
        Pendulum parameters :
            theta_min : -1.57079632679 [rad]
            g : 9.81 [N-m/s2]
            I : 0.333333333333 [kg-m**2]
            L : 0.5 [m]
            mass : 1.0 [kg]
            sin : <ufunc 'sin'> []
            theta_max : 1.57079632679 [rad]


    These parameter variables can then be called from within the class using
    for example:

        To assign a new value to the object variable from within the class:

        >>> self.g = 9.81 # Reassign gravity constant

        To assign to another variable from within the class:

        >>> example_g = self.g

    To call the parameters from outside the class, such as after instatiation
    similarly to the example above:

        To assign a new value to the object variable from outside the class:

        >>> pendulum_parameters = SystemParameters(L=1.0)
        >>> pendulum_parameters.L = 0.3 # Reassign length

        To assign to another variable from outside the class:

        >>> pendulum_parameters = SystemParameters()
        >>> example_g = pendulum_parameters.g # example_g = 9.81

    You can display the parameters using:

    >>> pendulum_parameters = SystemParameters()
    >>> print(pendulum_parameters.showParameters())
    Pendulum parameters :
        theta_min : -1.57079632679 [rad]
        g : 9.81 [N-m/s2]
        I : 0.333333333333 [kg-m**2]
        L : 0.5 [m]
        mass : 1.0 [kg]
        sin : <ufunc 'sin'> []
        theta_max : 1.57079632679 [rad]

    Or using biolog:

    >>> pendulum_parameters = SystemParameters()
    >>> biolog.info(system_parameters.showParameters())
    """

    def __init__(self, **kwargs):
        super(PendulumParameters, self).__init__('Pendulum')

        self.parameters = {}
        self.units = {}

        self.units['gravity'] = 'N-m/s2'
        self.units['length'] = 'm'
        self.units['theta_0'] = 'rad'
        self.units['theta_dot_0'] = 'rad/s'
        self.units['mass'] = 'kg'
        self.units['inertia'] = 'kg-m**2'
        self.units['origin_x'] = 'm'
        self.units['origin_y'] = 'm'

        # Pendulum parameters
        self.parameters['gravity'] = kwargs.pop(
            'gravity', 9.81)  # Gravity constant
        self.parameters['length'] = kwargs.pop('length', 0.3)  # Length
        self.parameters['mass'] = kwargs.pop('mass', 0.3)  # Pendulum Mass
        self.parameters['origin_x'] = kwargs.pop('origin_x', 0)
        self.parameters['origin_y'] = kwargs.pop('origin_y', 0)
        # Inertia
        self.set_inertia()
        self.parameters['theta_0'] = kwargs.pop(
            'theta_0', np.deg2rad(90))  # Pendulum initial angle
        self.parameters['theta_dot_0'] = kwargs.pop(
            'theta_dot_0', 0.)  # Pendulum initial velocity
        return

    @property
    def gravity(self):
        """ Get the value of gravity in the system. [N-m/s2]
        Default is 9.81 """
        return self.parameters['gravity']

    @gravity.setter
    def gravity(self, value):
        """ Keyword Arguments:
        value -- Set the value of gravity [N-m/s2] """
        self.parameters['gravity'] = value
        biolog.info(
            'Changed gravity to {} [N-m/s2]'.format(self.parameters['gravity']))

    @property
    def length(self):
        """ Get the value of pendulum length. [m]
        Default is 1.0"""
        return self.parameters['length']

    @length.setter
    def length(self, value):
        """ Keyword Arguments:
        value -- Set the value of pendulum's length [m] """
        self.parameters['length'] = value
        self.set_inertia()
        biolog.info(
            'Changed pendulum length to {} [m]'.format(self.parameters['length']))

    @property
    def mass(self):
        """Pendulum mass  """
        return self.parameters['mass']

    @mass.setter
    def mass(self, value):
        """Keyword Arguments:
           value --  Set the value of pendulum's mass [kg] """
        self.parameters['mass'] = value
        self.set_inertia()
        biolog.info('Changed pendulum mass to {} [kg]'.format(
            self.parameters['mass']))

    @property
    def inertia(self):
        """Pendulum inertia  """
        return self.parameters['inertia']

    @inertia.setter
    def inertia(self, _):
        raise Exception(
            'Cannot set inertianertia. Use mass and length properties to change inertia')

    @property
    def theta_0(self):
        """Pendulum theta_0  """
        return self.parameters['theta_0']

    @theta_0.setter
    def theta_0(self, value):
        self.parameters['theta_0'] = value

    @property
    def theta_dot_0(self):
        """Pendulum theta_dot_0  """
        return self.parameters['theta_dot_0']

    @theta_dot_0.setter
    def theta_dot_0(self, value):
        self.parameters['theta_dot_0'] = value

    @property
    def origin_x(self):
        """Pendulum origin_x  """
        return self.parameters['origin_x']

    @origin_x.setter
    def origin_x(self, value):
        self.parameters['origin_x'] = value

    @property
    def origin_y(self):
        """Pendulum origin_y  """
        return self.parameters['origin_y']

    @origin_y.setter
    def origin_y(self, value):
        self.parameters['origin_y'] = value

    def set_inertia(self):
        """Keyword Arguments:
           value --  Set the value of pendulum's inertia [kg] """
        self.parameters['inertia'] = self.mass * self.length**2 / 3.

    def showParameters(self):
        return self.msg(self.parameters, self.units)


if __name__ == '__main__':
    P = PendulumParameters(gravity=9.81, length=1.)
    print(P.showParameters())
