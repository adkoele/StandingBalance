#! /usr/bin/env python

""" Pendulum """

import itertools

import casadi as cas
import numpy as np

import biolog
from pendulum_parameters import PendulumParameters


class Pendulum(object):
    """Pendulum model main class."""

    def __init__(self, dae, parameters=PendulumParameters()):
        """ Initialize the pendulum system.
        Parameters
        ----------
        dae: <DaeGenerator>
            DaeGenerator instance for casadi integration.
        parameters: <PendulumParameters>
            Instance of PendulumParameters.
        """

        super(Pendulum, self).__init__()

        #: Dae generator
        self.dae = dae

        #: Integration options
        self.opts = None

        #: Integration states
        self.fin = {}

        #: Initialize pendulum states
        # theta
        self.theta = self.dae.add_x('theta', parameters.theta_0)
        # theta_dot
        self.theta_dot = self.dae.add_x('theta_dot',
                                        parameters.theta_dot_0)

        #: Initialize pendulum constants
        # Pendulum mass
        self.mass = self.dae.add_c('mass', parameters.mass)
        # Pendulum length
        self.length = self.dae.add_c('length', parameters.length)
        # Pendulum inertia
        self.inertia = self.dae.add_c('inertia', parameters.inertia)
        # System gravity
        self.gravity = self.dae.add_c('gravity', parameters.gravity)

        # Origin of the pendulum
        self.origin_x = self.dae.add_c('origin_x', parameters.origin_x)
        self.origin_y = self.dae.add_c('origin_y', parameters.origin_y)

        #: Initialize pendulum state derivatives
        # derivative of theta : Angular velocity
        # derivative of theta_d : Angular acceleration
        self.dtheta = self.dae.add_ode('dtheta', 0.0)
        self.dtheta_dot = self.dae.add_ode('dtheta_dot', 0.0)

        #: Initialize inputs to the pendulum
        # External torque
        self.torque = self.dae.add_u('torque', 0.0)

        #: Generate the ode
        self.ode_rhs()

        return

    def ode_rhs(self):
        """Pendulum System ODE rhs.
        Returns
        ----------
        ode_rhs: list<cas.SX>
            description
        """
        self.dtheta.sym = self.theta_dot.sym
        self.dtheta_dot.sym = -self.gravity.val * \
            self.mass.val * self.length.val * \
            cas.sin(self.theta.sym) / self.inertia.val + \
            self.torque.sym / self.inertia.val

    def pose(self):
        """Compute the full pose of the pendulum.

        Returns:
        --------
        pose: np.array
            [origin, center-of-mass]"""
        return np.array(
            [self.origin_x.val,
             self.origin_y.val + self.link_pose()])

    def link_pose(self):
        """ Position of the pendulum center of mass.

        Returns:
        --------
        link_pose: np.array
            Returns the current pose of pendulum COM"""

        #: pylint : disable=no-member
        return self.length.val * np.array([
            np.sin(self.theta.val),
            -np.cos(self.theta.val)])

    def generate_opts(self, opts):
        """ Generate options for integration."""
        if opts is not None:
            self.opts = opts
        else:
            self.opts = {'tf': 0.001,
                         'jit': False,
                         "print_stats": False,
                         "verbose": False}
        return

    #: pylint: disable=invalid-name
    def setup_integrator(self,
                         integration_method='cvodes',
                         opts=None):
        """Setup casadi integrator."""

        #: Generate Options for integration
        self.generate_opts(opts)
        #: Initialize states of the integrator
        self.fin['x0'] = self.dae.x
        self.fin['p'] = self.dae.u +\
            self.dae.p + self.dae.c
        self.fin['z0'] = self.dae.z
        self.fin['rx0'] = cas.DM([])
        self.fin['rp'] = cas.DM([])
        self.fin['rz0'] = cas.DM([])

        self.dae.print_dae()
        #: Set up the integrator
        self.integrator = cas.integrator('pendulum',
                                         integration_method,
                                         self.dae.generate_dae(),
                                         self.opts)
        return self.integrator

    def step(self, torque):
        """Step integrator."""
        self.torque.val = torque
        self.fin['p'][:] = list(itertools.chain(*self.dae.params))
        res = self.integrator.call(self.fin)
        self.fin['x0'][:] = res['xf'].full()[:, 0]
        self.fin['rx0'] = res['rxf']
        self.fin['rz0'] = res['rzf']
        return res


def main():
    """ Main """

    from musculo_skeletal_generator.dae_generator import DaeGenerator
    dae = DaeGenerator()
    P = Pendulum(dae)
    P.dae.print_dae()
    return


if __name__ == '__main__':
    main()
