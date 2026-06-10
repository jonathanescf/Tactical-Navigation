import numpy as np

from src.fleet.USV import USV

# Reference point for letlon to x y conversion (the current coordinates are Paris, but they can be changed to any other location)
LAT0 = 48.8566        
LON0 =  2.3522
METER_PER_DEG_LAT = 111320.0
METER_PER_DEG_LON = 111320.0 * np.cos(np.radians(LAT0))


class fake_USV_gps():
    def __init__(self,USV:USV):
        """
        This class simulates the gps sensor. It takes the USV state and converts the x y coordinates to latitude and longitude. It serves as an intermediate between the USV physics and the navigation algorithms that use GPS data.
        """
        self.USV = USV

    def _to_nmea(self, deg:float)->float:
        """
        conversion function used to convert the latitude and longitude in degrees to the NMEA format.
        """
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        return d * 100 + m
    
    def read_gll_non_busv_locking(self)->tuple[bool,list[float,str,float,str]]:
        """
        this function simulates the reading of the GPS data in NMEA format. Returns the latitude and longitude in NMEA format. The bool depicts the quality of the message, which is always good in this simulation. The altitude is set to 0.0 for now.
        """
        print("asking for the usv_lock in gll...")
        with self.USV.usv_lock:                          
            x, y = self.USV._x, self.USV._y
        print("getting out of the usv_lock")

        lat = LAT0 + y / METER_PER_DEG_LAT
        lon = LON0 + x / METER_PER_DEG_LON
        lat_nmea = self._to_nmea(lat)
        lon_nmea = self._to_nmea(abs(lon))
        hemi_lat = 'N' if lat >= 0 else 'S'
        hemi_lon = 'E' if lon >= 0 else 'W'
        return True, [lat_nmea, hemi_lat, lon_nmea, hemi_lon, 0.0]