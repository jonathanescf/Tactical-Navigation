import random

class mapbuilder:
    def __init__(self, parameters):
        self.waypoint = waypoint(parameters["nb_of_waypoints"], parameters["dict_of_waypoints"])
        self.obstacle = obstacle(parameters["nb_of_obstacles"], parameters["dict_of_obstacles"])

class waypoint:
    def __init__(self,nb_of_waypoints = 1, dict_of_waypoints = None):
        self.initialization(nb_of_waypoints, dict_of_waypoints)

    def initialization(self,nb_of_waypoints, dict_of_waypoints):
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
    def __init__(self,nb_of_obstacles = 0, dict_of_obstacles = None):
        self.initialization(nb_of_obstacles, dict_of_obstacles)

    def initialization(self,nb_of_obstacles, dict_of_obstacles):
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