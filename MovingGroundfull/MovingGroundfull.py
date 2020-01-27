"""This file implements reflex controller."""
# Default modules
import os
import numpy as np
import random as rd
from controller import Supervisor, Node

class MovingGround(Supervisor):
    """Main class for Human control. """

    def __init__(self):
        super(MovingGround, self).__init__()
        self.TIMESTEP = int(self.getBasicTimeStep())
        self.motor = self.getMotor("GROUND_GROUND_MOT")
        self.position_sensor = self.getPositionSensor("GROUND_GROUND_POS")
        self.position_sensor.enable(self.TIMESTEP)
        
        #load data
        load_data = np.loadtxt("Record0002.txt", comments="#")
        timevec = np.linspace(0,310,310*1000+1)
        time_old = load_data[:,0] - load_data[0,0]
        self.motion_data = np.interp(timevec, time_old, load_data[:,1])
        self.length = len(self.motion_data)
        
    def run(self):
        """ Run """
        counter = 0;
        #Random variable to start
        ind = 53719 #1
        des_pos = self.motion_data[0]
        time_elapsed = 0
        
        while self.step(self.TIMESTEP) != -1:   
            time_elapsed += self.TIMESTEP/1000.         
            
            # Only update position 1 in 3 time steps
            if counter > 0:
                des_pos = self.motion_data[ind]
                ind += 1
            
            if time_elapsed >= 2:
                counter = 1
            self.motor.setPosition(-des_pos)
            
                
def main():
    """ Main """
    moving_ground = MovingGround()
    moving_ground.run()

if __name__ == '__main__':
    main()
