### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.

import time
import numpy as np
from src.user_scripts.utilities.utils import *
from src.user_scripts.config import parameters
from src.user_scripts.src.USV_algos_for_motion_planning import USV_algos_for_motion_planning
from src.user_scripts.src.USV_control import USV_control
from src.user_scripts.src.USV_sensors import USV_sensors

from src.hardware.fake_USV_drivers.fake_USV_driver_arduino import fake_USV_ardu
from src.hardware.fake_USV_drivers.fake_USV_driver_gps import fake_USV_gps
from src.hardware.fake_USV_drivers.fake_USV_driver_imu import fake_USV_imu


class USV_master(USV_algos_for_motion_planning, USV_control, USV_sensors):
    def __init__(self, ardu:fake_USV_ardu, gps:fake_USV_gps, imu:fake_USV_imu):
        self.ardu = ardu
        self.gps = gps
        self.imu = imu
        self.running = False

    def test(self)->None:
        """
        Simple test function to test if the USV moves in the simulation
        """
        self.running = True
        Ul = 255
        Ur = 255
        print("Is the USV moving foward ?")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)
    
    def go_to_target(self, x_target:float, y_target:float)->None:
        
        self.running = True
        
        while self.running:

            x_USV, y_USV = self.current_x_y_position()

            if x_USV is not None and y_USV is not None:
                distance_to_target = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)

                # desired_heading = self.direct_waypoints(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_heading, _, _ = self.vector_field(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                desired_speed = 255 * np.tanh(distance_to_target)
                self.control_step(desired_speed, desired_heading)

                if distance_to_target < 10:
                    self.ardu.send_arduino_cmd_motor(140, 140)  

                if distance_to_target < 3:
                    self.ardu.send_arduino_cmd_motor(0, 0)  
                    break

                time.sleep(0.05)
         
    def go_to_waypoints(self, dict_waypoints:dict=False)->None:
        self.running = True

        if not dict_waypoints:
            dict_waypoints = parameters["dict_of_waypoints"]

        for i, (x_waypoint, y_waypoint) in list(dict_waypoints.items()):
            print(f"going to waypoint {i}: ({x_waypoint}, {y_waypoint})")
            self.go_to_target(x_target=x_waypoint, y_target=y_waypoint)
            del dict_waypoints[i]
            print(f"waypoint {i} reached")

        self.ardu.send_arduino_cmd_motor(0, 0)

    def wrapper(self):
        self.running = True
        mission = "wpts"
        
        while self.running ==True:
            if mission == "test":
                self.test()
            elif mission == "wpts":
                self.go_to_waypoints()