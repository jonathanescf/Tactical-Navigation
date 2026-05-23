import random

class mapbuilder:
    def __init__(self, nb_of_waypoints = 1, list_of_waypoints = None, nb_of_obstacles = 0, list_of_obstacles = None):
        self.waypoint = waypoint(nb_of_waypoints, list_of_waypoints)
        self.obstacle = obstacle(nb_of_obstacles, list_of_obstacles)

class waypoint:
    def __init__(self,nb_of_waypoints = 1, list_of_waypoints = None):
        self.initialization(nb_of_waypoints, list_of_waypoints)

    def initialization(self,nb_of_waypoints, list_of_waypoints):

        self.nb_of_waypoints = nb_of_waypoints

        if list_of_waypoints is not None:
            self.dico_of_waypoints = list_of_waypoints
        else:
            self.dico_of_waypoints = {}
            for i in range(1, self.nb_of_waypoints + 1):
                random_x = random.uniform(-80, 80)
                random_y = random.uniform(-80, 80)

                self.dico_of_waypoints[i] = (random_x, random_y)

class obstacle:
    def __init__(self,nb_of_obstacles = 0, list_of_obstacles = None):
        self.initialization(nb_of_obstacles, list_of_obstacles)

    def initialization(self,nb_of_obstacles, list_of_obstacles):
        self.nb_of_obstacles = nb_of_obstacles

        if list_of_obstacles is not None:
            self.dico_of_obstacles = list_of_obstacles
        else:
            self.dico_of_obstacles = {}
            for i in range(1, self.nb_of_obstacles + 1):
                random_x = random.uniform(-80, 80)
                random_y = random.uniform(-80, 80)
                radius = random.uniform(1, 5)

                self.dico_of_obstacles[i] = (random_x, random_y, radius)