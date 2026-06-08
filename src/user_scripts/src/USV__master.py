### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.


import numpy as np
import time
from src.user_scripts.utilities.utils import *
from src.user_scripts.config import parameters
from src.user_scripts.src.USV_algos_for_motion_planning import USV_algos_for_motion_planning
from src.user_scripts.src.USV_control import USV_control
from src.user_scripts.src.USV_sensors import USV_sensors


class USV_master(USV_algos_for_motion_planning, USV_control, USV_sensors):
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
    
    def go_to_target(self,x_target, y_target):
        
        self.running = True
        
        while self.running:
            x_USV, y_USV = self.current_x_y_position()

            if x_USV is not None and y_USV is not None:
                distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

                desired_heading = self.direct_waypoints(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                # desired_heading, _, _ = self.vector_field(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_speed = 255 * np.tanh(distance_to_target)
                self.control_step(desired_speed, desired_heading)

                if distance_to_target < 0.5:
                    self.ardu.send_arduino_cmd_motor(0, 0)  
                    break

    def go_to_waypoints(self,dict_waypoints=False):

        """
        The USV follows different waypoints from a dict.
        """
        self.running = True

        if not dict_waypoints:
            dict_waypoints = parameters["dict_of_waypoints"]
 
        for i,(x_waypoint,y_waypoint) in list(dict_waypoints.items()):   # copie des clés avant la boucle: on va supprimer les waypoints atteints du dico. changer la taille du dico sur lequel on itère casse le code
            x_USV, y_USV = self.current_x_y_position()
            x_target, y_target = x_waypoint, y_waypoint

            if x_USV is not None and y_USV is not None:
                distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

                # desired_heading = self.direct_waypoints(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_heading, _, _ = self.vector_field(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_speed = 255 * np.tanh(distance_to_target)
                self.control_step(desired_speed, desired_heading)

                if distance_to_target < 0.5:
                    del dict_waypoints[i]
                    self.ardu.send_arduino_cmd_motor(0, 0)  
                    break
            

            time.sleep(0.05)
        self.ardu.send_arduino_cmd_motor(0, 0)
         
    

    