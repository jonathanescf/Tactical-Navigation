import time
import numpy as np
from src.user_scripts.utilities.utils import *
from src.user_scripts.config import parameters
import matplotlib.pyplot as plt

def vector_field(x_USV, y_USV, x_target, y_target, dict_obstacles=parameters["dict_of_obstacles"]):
        k_attractive = 1
        k_repulsive = 1
        k_obstacle_influence = 3 # gain multiplied by the radius to have the surface in which the robot will be repulsed by the obstacle

        
        U_attractive = 0.5 * k_attractive * ((x_USV - x_target)**2 +(y_USV - y_target)**2) # we do pos - target because the potential vectors should point outwards from the target and have a positive value, that way when doing F = minus the gradient of U, we get a force that points towards the target.
        F_attractive = np.array([-k_attractive * (x_USV - x_target), -k_attractive * (y_USV - y_target)])
        
        if np.sqrt((x_USV-x_target)**2+(y_USV-y_target)**2) < 10:
            F_attractive *= 10

        U_repulsive = {}
        F_repulsive = {}
        
        for i, (x_obs, y_obs, r_obs) in dict_obstacles.items():
            d = np.sqrt((x_USV - x_obs)**2 + (y_USV - y_obs)**2) 
            d0 = k_obstacle_influence * r_obs # The obstacle influence is two times its size, this is can be changed to optimize the algorithm.
            if d < d0:

                ###* First Potential attempt: Using potentials found on articles
                U_repulsive[i] = 0.5 * k_repulsive * (1/d - 1/d0)**2 * d0**4 # the last term d0 ** 4 is to make both attrctive terms and repulsive terms equivalent to d. ( Otherwise, the order of magnitude is not the same at all)
   
                #* We use the chain rule here (Document the math in the README)
                diff_x = x_USV - x_obs
                diff_y = y_USV - y_obs
                d_hat = np.array([diff_x, diff_y])  / d # direction of the repulsive force
                F_repuls_x = k_repulsive * (1/d - 1/d0) * (1/d**2) * d_hat[0] *d0**4  # the last term d0 ** 4 is to make both attrctive terms and repulsive terms equivalent to d. ( Otherwise, the order of magnitude is not the same at all)
                F_repuls_y = k_repulsive * (1/d - 1/d0) * (1/d**2) * d_hat[1] *d0**4  # the last term d0 ** 4 is to make both attrctive terms and repulsive terms equivalent to d. ( Otherwise, the order of magnitude is not the same at all)
                F_repulsive[i] = np.array([F_repuls_x, F_repuls_y])

                ###* Second attempt: choosing my own potentials
                
                # U_repulsive[i] = (d0 - d) / (d - r_obs)**2

                # diff_x = x_USV - x_obs
                # diff_y = y_USV - y_obs
                # d_hat = np.array([diff_x, diff_y]) / d

                # dU_dd = (d - 2*d0 + r_obs) / (d - r_obs)**3
                # F_repulsive[i] = -k_repulsive * dU_dd * d_hat
                

            else:
               U_repulsive[i] = 0
               F_repulsive[i] = np.array([0, 0])

        U_total = U_attractive + sum(U_repulsive.values())
        F_total = np.array([F_attractive[0] + sum(F_repulsive[i][0] for i in F_repulsive), F_attractive[1] + sum(F_repulsive[i][1] for i in F_repulsive)])

        return F_total, U_repulsive, k_obstacle_influence # 2nd argument for plotting purposes


class USV_algorithms:
    def cap_to(self, psi_desired_deg, v_desired = 255):
        """
        Sets the USV to a certain heading
        """
        self.running = True
        while self.running:
            self.control_step(v_desired, np.radians(psi_desired_deg))
            time.sleep(0.05)

    def go_to_waypoints(self,dict_waypoints=False,order=False):

        """
        The USV follows different waypoints from a dict.
        """
        self.running = True

        if not dict_waypoints:
            dict_waypoints = parameters["dict_of_waypoints"]

        if order == False:
            L = list(dict_waypoints.keys())
    
        # if order==False: # We order the waypoints by distance   
        for i,(x_waypoint,y_waypoint) in list(dict_waypoints.items()):   # copie des clés avant la boucle: on va supprimer les waypoints atteints du dico. changer la taille du dico sur lequel on itère casse le code
            while self.running:
                x_USV, y_USV = self.current_x_y_position()

                if x_USV is not None and y_USV is not None:
                    x_target, y_target = x_waypoint, y_waypoint
                    dist = np.sqrt((x_target - x_USV)**2 + (y_target - y_USV)**2)
                    
                    # # Go to waypoints without waypoints 
                    # desired_heading = np.arctan2(y_target - y_USV, x_target - x_USV)
                    
                    vector_field_output, _, _ = vector_field(x_USV = x_USV, y_USV = y_USV, x_target = x_target, y_target = y_target)
                    desired_heading = vector_to_heading(vector_field_output)

                    desired_speed = 255 * np.tanh(dist)
                    self.control_step(desired_speed, desired_heading)

                    # print(f"Current heading: {np.degrees(self.current_heading())} degrees, Desired heading: {np.degrees(desired_heading)} degrees")

                    if dist < 0.5:  
                        print(f"Waypoint {i} reached.")
                        del dict_waypoints[i]
                        break

                time.sleep(0.05)
        self.ardu.send_arduino_cmd_motor(0, 0)
