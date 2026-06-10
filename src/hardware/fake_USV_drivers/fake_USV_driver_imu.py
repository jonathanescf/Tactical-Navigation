import numpy as np
from src.fleet.USV import USV



class fake_USV_imu:
    def __init__(self, USV:USV):
        """
        This is a fake IMU driver that simulates magnetometer readings based on the USV's heading (psi).
        """
        self.USV = USV

    def read_mag_raw(self)->list[float,float,float]:
        """
        simulates magnetometer readings based on the USV's heading (psi).
        """
        psi = self.USV.psi_rad

        bx =  np.cos(psi) * 10000
        by = -np.sin(psi) * 10000
        bz = 0.0
        return [bx, by, bz]