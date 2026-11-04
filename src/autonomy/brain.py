### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.

import time
import numpy as np
from src.autonomy.utils import *
from src.orchestration.config import parameters
from src.autonomy.guidance import USV_Guidance
from src.autonomy.control import USV_Control
from src.autonomy.navigation import USV_Navigation
from src.autonomy.mission import USV_Mission

from src.simulation.fleet.hardware.fake_USV_motor_driver import fake_USV_ardu
from src.simulation.fleet.hardware.fake_GPS_driver import fake_USV_gps
from src.simulation.fleet.hardware.fake_IMU_driver import fake_USV_imu


class USV_brain(USV_Mission, USV_Guidance, USV_Navigation, USV_Control):
    def __init__(self, ardu:fake_USV_ardu, gps:fake_USV_gps, imu:fake_USV_imu):
        self.ardu = ardu
        self.gps = gps
        self.imu = imu
        self.ongoing_target_state = None, None, None, None
        self.waypoints = list(parameters["dict_of_waypoints"].items())
        self.wp_index = 0

    def target_reached(self):
        self.wp_index += 1

    def next_target(self):
        if self.wp_index >= len(self.waypoints):
            return None
        return self.waypoints[self.wp_index][1]

    def brain_step(self):
        nav_state = self.get_nav_state()
        self.ongoing_target_state = self.next_target()

        if self.ongoing_target_state is None:
            self.ardu.send_arduino_cmd_motor(0, 0)
            return

        Ul, Ur, TargetIntercepted = self.go_to_target_step(nav_state, self.ongoing_target_state)
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

        if TargetIntercepted:
            self.target_reached()
            print("Target reached")
        return


    