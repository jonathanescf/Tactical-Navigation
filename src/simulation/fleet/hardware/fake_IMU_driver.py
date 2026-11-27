import numpy as np


#TODO: ajouter du bruit aux filtres pour les rendre plus réalistes
class fake_USV_imu:
    def __init__(self, USV_body, div, dt):
        """
        This is a fake IMU driver that simulates magnetometer readings based on the USV's heading (psi).
        """
        self.USV_body = USV_body
        self.div = div
        self.dt = dt
        self.latest = None

    def read_mag_raw(self, k)->list[float,float,float]:
        """
        simulates magnetometer readings based on the USV's heading (psi).
        """
        psi = self.USV_body.psi_rad

        bx =  np.cos(psi) * 10000
        by = -np.sin(psi) * 10000
        bz = 0.0
        return [bx, by, bz], k * self.dt

    def update_imu_data(self, k):
        if k % self.div == 0:
            self.latest = self.read_mag_raw(k)

    def get_imu_data(self):
        return self.latest