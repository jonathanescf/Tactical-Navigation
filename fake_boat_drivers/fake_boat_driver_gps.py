import numpy as np


# Reference point for letlon to x y conversion (the current coordinates are Paris, but they can be changed to any other location)
LAT0 = 48.8566        
LON0 =  2.3522
METER_PER_DEG_LAT = 111320.0
METER_PER_DEG_LON = 111320.0 * np.cos(np.radians(LAT0))


class fake_boat_gps():
    def __init__(self,boat):
        """
        This class simulates the gps sensor. It takes the boat state and converts the x y coordinates to latitude and longitude. It serves as an intermediate between the boat physics and the navigation algorithms that use GPS data.
        """
        self.boat = boat

    def _to_nmea(self, deg):
        """
        conversion function used to convert the latitude and longitude in degrees to the NMEA format.
        """
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        return d * 100 + m
    
    def read_gll_non_blocking(self):
        """
        this function simulates the reading of the GPS data in NMEA format. Returns the latitude and longitude in NMEA format. The bool depicts the quality of the message, which is always good in this simulation. The altitude is set to 0.0 for now.
        """
        lat = LAT0 + self.boat.y / METER_PER_DEG_LAT
        lon = LON0 + self.boat.x / METER_PER_DEG_LON
        lat_nmea = self._to_nmea(lat)
        lon_nmea = self._to_nmea(abs(lon))
        hemi_lat = 'N' if lat >= 0 else 'S'
        hemi_lon = 'E' if lon >= 0 else 'W'
        return True, [lat_nmea, hemi_lat, lon_nmea, hemi_lon, 0.0]