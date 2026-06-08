import time
import numpy as np
from src.user_scripts.utilities.utils import *
from src.user_scripts.config import parameters
import matplotlib.pyplot as plt



class USV_algos_for_motion_planning:
    def cap_to(self, psi_desired_deg, v_desired = 255):
        """
        Sets the USV to a certain heading
        """
        self.running = True
        while self.running:
            self.control_step(v_desired, np.radians(psi_desired_deg))
            time.sleep(0.05)

    def direct_waypoints(self, x_USV, y_USV, x_target, y_target):
        vector = np.array([x_target - x_USV, y_target - y_USV])
        desired_heading = vector_to_heading(vector)
        return desired_heading

    def vector_field(self, x_USV, y_USV, x_target, y_target, dict_obstacles=parameters["dict_of_obstacles"]):
        k_attractive = 1
        k_repulsive = 2
        k_obstacle_influence = 3 # gain multiplied by the radius to have the surface in which the robot will be repulsed by the obstacle

        U_dict = {}
        F_dict = {}
        U_dict["target"] = 0.5 * k_attractive * ((x_USV - x_target)**2 +(y_USV - y_target)**2) # we do pos - target because the potential vectors should point outwards from the target and have a positive value, that way when doing F = minus the gradient of U, we get a force that points towards the target.
        F_dict["target"] = np.array([-k_attractive * (x_USV - x_target), -k_attractive * (y_USV - y_target)])
        
        if np.sqrt((x_USV-x_target)**2+(y_USV-y_target)**2) < 20:
            F_dict["target"] *= 10
        
        for i, (x_obs, y_obs, r_obs) in dict_obstacles.items():
            
            d = np.sqrt((x_USV - x_obs)**2 + (y_USV - y_obs)**2) 
            d0 = k_obstacle_influence * r_obs # The obstacle influence is two times its size, this is can be changed to optimize the algorithm.
            
            if d < d0:

                ###* First Potential attempt: Using potentials found on articles
                U_dict[f"obstacle {i}"] = 0.5 * k_repulsive * (1/d - 1/d0)**2 * d0 ** 4
   
                #* We use the chain rule here (Document the math in the README)
                diff_x = x_USV - x_obs
                diff_y = y_USV - y_obs
                d_hat = np.array([diff_x, diff_y])  / d # direction of the repulsive force
                F_repuls_x = k_repulsive * (1/d - 1/d0) * (1/d**0.75) * d_hat[0] * d0 ** 2.75 # the last term d0 ** 2.75 is to make both attrctive terms and repulsive terms equivalent to d. ( Otherwise, the order of magnitude is not the same at all)
                F_repuls_y = k_repulsive * (1/d - 1/d0) * (1/d**0.75) * d_hat[1] * d0 ** 2.75 # the last term d0 ** 2.75 is to make both attrctive terms and repulsive terms equivalent to d. ( Otherwise, the order of magnitude is not the same at all)
                F_dict[f"obstacle {i}"] = np.array([F_repuls_x, F_repuls_y])

            else:
               U_dict[f"obstacle {i}"] = 0
               F_dict[f"obstacle {i}"] = np.array([0, 0])

        U_total = sum(U_dict.values())
        F_total = np.array([sum(F_dict[i][0] for i in F_dict), sum(F_dict[i][1] for i in F_dict)])

        desired_heading = vector_to_heading(F_total)

        return desired_heading, U_dict, k_obstacle_influence # 2nd and 3rd arguments for plotting purposes

       