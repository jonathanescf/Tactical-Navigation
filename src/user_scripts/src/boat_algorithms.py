import time
import numpy as np
from src.user_scripts.utilities.utils import *
from src.user_scripts.config import parameters

class boat_algorithms:
    def cap_to(self, psi_desired_deg, v_desired = 255):
        """
        Sets the boat to a certain heading
        """
        self.running = True
        while self.running:
            self.control_step(v_desired, np.radians(psi_desired_deg))
            time.sleep(0.05)

    def go_to_waypoints(self,dict_waypoints=False,order=False):
        """
        The boat follows different waypoints from a dict.
        """
        self.running = True

        if not dict_waypoints:
            dict_waypoints = parameters["dict_of_waypoints"]

        if order == False:
            L = list(dict_waypoints.keys())
    
        # if order==False: # We order the waypoints by distance   
        for i,(x_waypoint,y_waypoint) in list(dict_waypoints.items()):   # copie des clés avant la boucle: on va supprimer les waypoints atteints du dico. changer la taille du dico sur lequel on itère casse le code
            while self.running:
                x, y = self.current_x_y_position()

                if x is not None and y is not None:
                    target_x, target_y = x_waypoint, y_waypoint
                    dist = np.sqrt((target_x - x)**2 + (target_y - y)**2)
                    desired_heading = np.arctan2(target_y - y, target_x - x)
                    desired_speed = 255 * np.tanh(dist)
                    self.control_step(desired_speed, desired_heading)

                    if dist < 0.5:  
                        print(f"Waypoint {i} reached.")
                        del dict_waypoints[i]
                        break

                time.sleep(0.05)
        self.ardu.send_arduino_cmd_motor(0, 0)

    def vector_field(self, x_target, y_target, dict_obstacles=False):
        pass