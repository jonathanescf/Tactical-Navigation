from src.boat import boat



class fleet_manager:
    def __init__(self, parameters):
        self.nb_boats = parameters["nb_of_boats"]
        self.initialization()
        

    def initialization(self):
        self.boat = {}
        for id in range (1,self.nb_boats+1):
            self.boat[id] = boat(id)
            


        