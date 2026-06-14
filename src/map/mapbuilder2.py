import numpy as np
import random
import matplotlib.pyplot as plt

class mapbuilder2:
    def __init__(self,parameters, grid_positive_length=300):
        self.waypoint = waypoint(parameters["nb_of_waypoints"], parameters["dict_of_waypoints"])
        self.obstacle = obstacle(parameters["nb_of_obstacles"], parameters["dict_of_obstacles"])
        self.grid_positive_length = grid_positive_length
        self.grid_length = grid_positive_length * 2
        self.grid_shape = (self.grid_length, self.grid_length)

    def create_map(self):
        cx = self.grid_positive_length
        cy = self.grid_positive_length

        map_array = np.zeros(self.grid_shape)

        for i in range(self.grid_length):
            for j in range(self.grid_length):

                x = i - cx
                y = j - cy

                r = np.sqrt(x**2 + y**2)

                if 50 < r < 100:
                    map_array[i][j] = 1

                if 150 < r < 200:
                    map_array[i][j] = 1

                if 250 < r < 300:
                    map_array[i][j] = 1

                if np.abs(x - y) < 25:
                    map_array[i][j] = 1

        # plt.imshow(map_array, cmap='gray', vmin=0, vmax=1)
        # plt.axis('off')
        # plt.show()
        return map_array

    
class waypoint:
    def __init__(self,nb_of_waypoints:int = 1, dict_of_waypoints:dict = None):
        self.initialization(nb_of_waypoints, dict_of_waypoints)

    def initialization(self,nb_of_waypoints:int, dict_of_waypoints:dict):
        if dict_of_waypoints is not None:
            self.nb_of_waypoints = len(dict_of_waypoints)
            self.dico_of_waypoints = dict_of_waypoints
        else:
            self.nb_of_waypoints = nb_of_waypoints
            self.dico_of_waypoints = {}
            for i in range(1, self.nb_of_waypoints + 1):
                random_x = random.uniform(-80, 80)
                random_y = random.uniform(-80, 80)

                self.dico_of_waypoints[i] = (random_x, random_y)

class obstacle:
    def __init__(self,nb_of_obstacles:int = 0, dict_of_obstacles:dict = None):
        self.initialization(nb_of_obstacles, dict_of_obstacles)

    def initialization(self,nb_of_obstacles:int, dict_of_obstacles:dict):
        if dict_of_obstacles is not None:
            self.nb_of_obstacles = len(dict_of_obstacles)
            self.dico_of_obstacles = dict_of_obstacles
        else:
            self.nb_of_obstacles = nb_of_obstacles
            self.dico_of_obstacles = {}
            for i in range(1, self.nb_of_obstacles + 1):
                random_x = random.uniform(-80, 80)
                random_y = random.uniform(-80, 80)
                radius = random.uniform(1, 5)

                self.dico_of_obstacles[i] = (random_x, random_y, radius)


if __name__ == "__main__":
    m = mapbuilder2()
    m.create_map()

        
