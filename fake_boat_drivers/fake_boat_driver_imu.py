# fake_drivers/fake_imu.py
import numpy as np

class fake_boat_imu:
    def __init__(self, ddboat_state):
        self.boat = ddboat_state

    def read_mag_raw(self):

        psi = self.boat.psi_rad

        bx =  np.cos(psi) * 10000
        by = -np.sin(psi) * 10000
        bz = 0.0
        return [bx, by, bz]