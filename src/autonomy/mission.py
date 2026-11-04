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
        while True:
            self.control_step(v_desired, np.radians(psi_desired_deg))
            time.sleep(0.05)
    
    def go_to_target(self, x_target:float, y_target:float)->None:
        
        while True:

            x_USV, y_USV, psi_USV, v_USV = self.get_nav_state()

            if x_USV is not None and y_USV is not None:
                distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

                # desired_heading = self.direct_waypoints(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_heading, _, _ = self.vector_field(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_speed = 255 * np.tanh(distance_to_target)

                Ul, Ur = self.controller(v_USV, psi_USV, desired_speed, desired_heading)
                self.ardu.send_arduino_cmd_motor(Ul, Ur)

                if distance_to_target < 3:
                    self.ardu.send_arduino_cmd_motor(0, 0)  
                    break

                time.sleep(0.05)

    def go_to_target_step(self, nav_state, target_state, interception_accuracy=3):

        x_USV, y_USV, psi_USV, v_USV = nav_state
        x_target, y_target, psi_target, v_target = target_state

        distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

        desired_speed = self.get_desired_speed(distance_to_target)
        desired_heading = self.get_desired_heading(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)

        Ul, Ur = self.controller(v_USV, psi_USV, desired_speed, desired_heading)

        if distance_to_target > interception_accuracy:
            return Ul, Ur, False
        else:
            return 0, 0, True








         
    def go_to_waypoints(self, dict_waypoints:dict=False)->None:

        if not dict_waypoints:
            dict_waypoints = parameters["dict_of_waypoints"]

        for i, (x_waypoint, y_waypoint) in dict_waypoints.items():
            self.ongoing_target = (x_waypoint, y_waypoint)
            print(f"going to waypoint {i}: ({x_waypoint}, {y_waypoint})")
            self.go_to_target(x_target=x_waypoint, y_target=y_waypoint)
            print(f"waypoint {i} reached")

        self.ardu.send_arduino_cmd_motor(0, 0)