#! /usr/bin/env python
""" Generate a pendulum model example to test the musculoskeletalsystem."""

import numpy as np

from musculo_skeletal_generator.dae_generator import DaeGenerator
from musculo_skeletal_generator.musculo_skeletal_generator import \
    MusculoSkeletalSystem
from pendulum_model import Pendulum
from matplotlib import pyplot as plt
import biolog


def main():
    """Main"""
    dae = DaeGenerator()
    pendulum = Pendulum(dae)
    #: opts
    dt = 0.001
    opts = {'tf': dt,
            'jit': False,
            "enable_jacobian": True,
            "print_time": False,
            "print_stats": False,
            "reltol": 1e-6,
            "abstol": 1e-6}
    pendulum.setup_integrator(opts=opts)
    pendulum.dae.print_dae()

    musculo_sys = MusculoSkeletalSystem(
        './conf/pendulum_config.yaml', opts=opts)

    #: Initialize network parameters
    #: pylint: disable=invalid-name
    _time = np.arange(0, 1.5, dt)  #: Time
    musculo_sys.muscle_sys.activations['flexor'] = 0.05
    musculo_sys.muscle_sys.activations['extensor'] = 0.5
    res = []
    res1 = []
    res2 = []
    for t in _time:
        #: Update the musculo-skeletal system
        musculo_sys.joint_sys.positions['joint1'] = pendulum.dae.x[0]
        res.append(musculo_sys.joint_sys.positions['joint1'])
        res1.append(
            musculo_sys.muscle_sys.muscles['flexor'].l_mtc)
        res2.append(
            musculo_sys.muscle_sys.muscles['extensor'].l_mtc)
        musculo_sys.step()
        pendulum.step(
            musculo_sys.joint_sys.joints['joint1'].torque)

    plt.subplot(211)
    plt.plot(res, res1)
    plt.plot(res, res2)
    plt.grid()
    plt.subplot(212)
    plt.plot(_time, np.rad2deg(res))
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
