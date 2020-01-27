#!/usr/bin/python
""" Generate Isomertic Muscle Data."""
import time

import matplotlib.pyplot as plt
import numpy as np

import biolog
from musculo_skeletal_generator.musculo_skeletal_generator import MusculoSkeletalSystem
from musculo_skeletal_generator.musculo_skeletal_parameters import MuscleParameters

# Global settings for plotting
# You may change as per your requirement
plt.rc('lines', linewidth=2.0)
plt.rc('font', size=12.0)
plt.rc('axes', titlesize=14.0)     # fontsize of the axes title
plt.rc('axes', labelsize=14.0)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=14.0)    # fontsize of the tick labels
plt.rc('ytick', labelsize=14.0)    # fontsize of the tick labels


def isometric_contraction():
    iso_sys = MusculoSkeletalSystem(
        '../../../musculo_skeletal_generator/conf/muscles.yaml')

    #: Initialize network parameters
    #: pylint: disable=invalid-name
    dt = 1  #: Time step
    _time = np.arange(0, 100, dt)  #: Time

    #: Vector to store results
    res = np.empty([len(_time), len(iso_sys.muscle_sys.dae.x)])
    res_deep = np.empty([len(_time), len(iso_sys.muscle_sys.dae.z)])

    muscle_stretch = np.concatenate(
        (np.linspace(0.0, -0.05, 25), np.linspace(-0.1, 0.05, 50)))

    length = np.empty(np.shape(muscle_stretch))
    force = np.empty(np.shape(muscle_stretch))
    active_force = np.empty(np.shape(muscle_stretch))
    passive_force = np.empty(np.shape(muscle_stretch))
    start_time = time.time()

    #: Integrate the network
    muscle_act={'m1':0.75}
    for idxx, stretch in enumerate(muscle_stretch):
        iso_sys.muscle_sys.dae.p[0] = stretch
        for idx, _ in enumerate(_time):
            iso_sys.step(muscle_act)
            res[idx] = iso_sys.state['muscles']['xf'].full()[:, 0]
            res_deep[idx] = iso_sys.state['muscles']['zf'].full()[
                :, 0]
        length[idxx] = res[-1, 0]
        force[idxx] = res_deep[-1, 0]
        active_force[idxx] = res_deep[-1, 1]
        passive_force[idxx] = res_deep[-1, 2]
    l_opt = iso_sys.muscle_sys.muscles['m1'].l_opt.val
    f_max = iso_sys.muscle_sys.muscles['m1'].f_max.val

    end_time = time.time()

    biolog.info('Execution Time : {}'.format(
        end_time - start_time))

    # Plotting
    plt.figure('2a Force-Length')
    plt.plot(length / l_opt, force)
    plt.plot(length / l_opt, active_force)
    plt.plot(length / l_opt, passive_force)
    plt.plot(np.ones((25, 1)), np.linspace(0., f_max, 25),
             '--')
    plt.text(0.6, 600, 'Below \n optimal \n length',
             fontsize=14)
    plt.text(1.1, 600, 'Above \n optimal \n length',
             fontsize=14)
    plt.title('Force-Length Relationship')
    plt.xlabel('Muscle CE Length')
    plt.ylabel('Muscle Force')
    plt.legend(('Force', 'Active Force', 'Passive Force'), loc=2)
    plt.grid()
    plt.show()


if __name__ == '__main__':
    isometric_contraction()
