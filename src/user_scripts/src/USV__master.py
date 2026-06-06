### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.



import numpy as np
from src.user_scripts.utilities.utils import *

from src.user_scripts.src.USV_algorithms import USV_algorithms
from src.user_scripts.src.USV_control import USV_control
from src.user_scripts.src.USV_sensors import USV_sensors


class USV_master(USV_algorithms, USV_control, USV_sensors):
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
        print("Is the USV moving foward ?")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

         
    

    