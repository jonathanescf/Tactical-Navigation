import numpy as np
from src.utils import *
import time


class ddboat_mission:
    def __init__(self, waypoint, ardu, gps, imu):
        self.waypoint = waypoint
        self.ardu = ardu
        self.gps = gps
        self.imu = imu
        self.running = False

    def test(self):
        self.running = True
        Ul = 255
        Ur = 255
        print("Is the boat moving foward ?")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

    def current_heading(self):
        mag = self.imu.read_mag_raw()
        return -np.arctan2(mag[1], mag[0])
    
    def current_speed(self):            ### Implémentez un filtre de kalman sur l'imu et le gps pour estimer la vitesse du bateau
        K = 0.05
        Ul, Ur = self.ardu.get_arduino_cmd_motor()
        v     = K * (Ul + Ur) / 2
        return v
        
    def current_lat_lon_position(self):
        msg, val = self.gps.read_gll_non_blocking()
        if msg:
            lat = nmea_to_decimal(val[0])   # conversion NMEA → degrés
            lon = nmea_to_decimal(val[2])
            if val[1] == 'S': lat = -lat
            if val[3] == 'W': lon = -lon
            return lat, lon
        return None, None
        
    def current_x_y_position(self):
        lat, lon = self.current_lat_lon_position()
        if lat is not None and lon is not None:
            x, y = latlon_to_xy(lat, lon)
        else:
            x, y = None, None
        return x, y
        
    def control_law(self, Vd, Wd):
        """ Compute the command voltages for the left and right motors based on the desired linear velocity (Vd) and angular velocity (Wd) using a feed-forward control approach. """
        Kp  = 200          
        Ur  = Vd + Kp * (Wd / np.pi)
        Ul  = Vd - Kp * (Wd / np.pi)

        return Ur, Ul
    
    def control_step(self, v_desired, psi_desired_rad):
        delta_w =  -sawtooth(self.current_heading() - psi_desired_rad)
        Ur, Ul = self.control_law(v_desired, delta_w)
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

    def cap_to(self, psi_desired_deg, v_desired = 255):
        self.running = True
        while self.running:
            self.control_step(v_desired, np.radians(psi_desired_deg))
            time.sleep(0.05)

    def go_to_waypoints(self,dico_waypoints):
        self.running = True
        for i in list(dico_waypoints.keys()):   # copie des clés avant la boucle: on va supprimer les waypoints atteints du dico. changer la taille du dico sur lequel on itère casse le code

            while self.running:
                x, y = self.current_x_y_position()

                if x is not None and y is not None:
                    target_x, target_y = self.waypoint.dico_of_waypoints[i]
                    dist = np.sqrt((target_x - x)**2 + (target_y - y)**2)
                    desired_heading = np.arctan2(target_y - y, target_x - x)
                    desired_speed = 255 * np.tanh(dist)
                    self.control_step(desired_speed, desired_heading)

                    print(f"Current position: ({x:.2f}, {y:.2f}), Target: ({target_x:.2f}, {target_y:.2f}), Distance: {dist:.2f}")

                    if dist < 0.5:  
                        print(f"Waypoint {i} reached.")
                        del dico_waypoints[i]
                        break

                time.sleep(0.05)
        self.ardu.send_arduino_cmd_motor(0, 0)


            