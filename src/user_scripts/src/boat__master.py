### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.



import numpy as np
from src.user_scripts.utilities.utils import *

from src.user_scripts.src.boat_algorithms import boat_algorithms
from src.user_scripts.src.boat_control import boat_control
from src.user_scripts.src.boat_sensors import boat_sensors


class boat_master(boat_algorithms, boat_control, boat_sensors):
    def __init__(self, ardu, gps, imu):
        self.ardu = ardu
        self.gps = gps
        self.imu = imu
        self.running = False

    def test(self):
        """
        Simple test function to test if the USV moves in the simulation
        """
        self.running = True
        Ul = 255
        Ur = 255
        print("Is the boat moving foward ?")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

         
    

    