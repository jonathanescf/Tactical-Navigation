from src.simulation.fleet.vehicles.USV import USV



class simulator:
    def __init__(self, parameters:dict, terrain=None):
        self.nb_USVs = parameters["nb_of_USVs"]
        self.initialization(terrain)
        

    def initialization(self, terrain):
        self.dict_of_assets = {}
        self.list_of_assets=[]
        for num in range (1,self.nb_USVs+1):
            id = "usv" + str(num)
            self.dict_of_assets[id] = USV(id = id, 
                                          terrain = terrain)
            self.list_of_assets.append(self.dict_of_assets[id])




        