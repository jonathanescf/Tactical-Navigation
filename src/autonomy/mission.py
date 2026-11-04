import time
import numpy as np
from src.autonomy.utils import *
from src.orchestration.config import parameters


class USV_Mission():

    def test(self)->None:
        """
        Simple test function to test if the USV moves in the simulation
        """
        Ul = 120
        Ur = 255
        print("Is the USV moving foward ?")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

    def cap_to(self, psi_desired_deg:float, v_desired:float = 255)->None:
        """
        Sets the USV to a certain heading
        """
        Ul, Ur = self.controller(v_desired, np.radians(psi_desired_deg))
        return Ul, Ur
    
    def go_to_target_step(self, nav_state, target_state, 
                          dict_obstacles, interception_accuracy=3):

        x_USV, y_USV, psi_USV, v_USV = nav_state
        x_target, y_target, psi_target, v_target = target_state

        distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

        desired_speed = self.get_desired_speed(distance_to_target)
        desired_heading = self.get_desired_heading(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target, dict_obstacles= dict_obstacles)

        Ul, Ur = self.controller(v_USV, psi_USV, desired_speed, desired_heading)

        if distance_to_target > interception_accuracy:
            return Ul, Ur, False
        else:
            return 0, 0, True