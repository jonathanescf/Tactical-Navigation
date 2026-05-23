import numpy as np

LAT0 = 48.8566        # Latitude de référence (ex: Paris)
LON0 =  2.3522
METER_PER_DEG_LAT = 111320.0
METER_PER_DEG_LON = 111320.0 * np.cos(np.radians(LAT0))


class fake_boat_gps():
    def __init__(self,ddboat_state):
        self.boat = ddboat_state

    def _to_nmea(self, deg):
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        return d * 100 + m
    
    def read_gll_non_blocking(self):
        lat = LAT0 + self.boat.y / METER_PER_DEG_LAT
        lon = LON0 + self.boat.x / METER_PER_DEG_LON
        lat_nmea = self._to_nmea(lat)
        lon_nmea = self._to_nmea(abs(lon))
        hemi_lat = 'N' if lat >= 0 else 'S'
        hemi_lon = 'E' if lon >= 0 else 'W'
        return True, [lat_nmea, hemi_lat, lon_nmea, hemi_lon, 0.0]