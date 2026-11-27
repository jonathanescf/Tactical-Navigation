import numpy as np
import random
from src.orchestration.config import LAT0, LON0, METER_PER_DEG_LAT, METER_PER_DEG_LON


#TODO: ajouter du bruit aux filtres pour les rendre plus réalistes
class fake_USV_gps():
    def __init__(self,USV_body, div, dt):
        """
        This class simulates the gps sensor. It takes the USV_body state and converts the x y coordinates to latitude and longitude. It serves as an intermediate between the USV_body physics and the navigation algorithms that use GPS data.
        """
        self.USV_body = USV_body
        self.div = div
        self.dt = dt
        self.latest = None
        self.t_latest = None

    def _to_nmea(self, deg:float)->float:
        """
        conversion function used to convert the latitude and longitude in degrees to the NMEA format.
        """
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        return d * 100 + m
    
    def read_gll_non_blocking(self, k)->tuple[bool,list[float,str,float,str]]:
        """
        this function simulates the reading of the GPS data in NMEA format. Returns the latitude and longitude in NMEA format. The bool depicts the quality of the message, which is always good in this simulation. The altitude is set to 0.0 for now.
        """
        with self.USV_body.usv_lock:                          
            x, y = self.USV_body.x, self.USV_body.y  

        lat = LAT0 + y / METER_PER_DEG_LAT
        lon = LON0 + x / METER_PER_DEG_LON
        lat_nmea = self._to_nmea(lat)
        lon_nmea = self._to_nmea(abs(lon))
        hemi_lat = 'N' if lat >= 0 else 'S'
        hemi_lon = 'E' if lon >= 0 else 'W'
        
        return True, [lat_nmea, hemi_lat, lon_nmea, hemi_lon, 0.0], k * self.dt

    def update_gps_data(self, k):
        if k % self.div == 0:
            self.t_latest = k * self.dt
            self.latest = self.read_gll_non_blocking(k) # giving the simulation kick for timestamp feeedback
    
    def get_gps_data(self):
        return self.latest 