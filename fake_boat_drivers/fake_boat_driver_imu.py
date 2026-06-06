import numpy as np



class fake_boat_imu:
    def __init__(self, ddboat_state):
        """
        This is a fake IMU driver that simulates magnetometer readings based on the boat's heading (psi).
        """
        self.boat = ddboat_state

    def read_mag_raw(self):
        """
        simulates magnetometer readings based on the boat's heading (psi).
        """
        psi = self.boat.psi_rad

        bx =  np.cos(psi) * 10000
        by = -np.sin(psi) * 10000
        bz = 0.0
        return [bx, by, bz]