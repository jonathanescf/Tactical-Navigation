from src.simulation.fleet.vehicles.USV import USV



class simulator:
    def __init__(self, parameters:dict, mission_info:dict, terrain=None, world_model=None):
        self.nb_USVs = parameters["nb_of_USVs"]
        self.taken_ids = []
        self.initialization(mission_info, terrain, world_model)
        

    def initialization(self, mission_info, terrain, world_model):
        self.dict_of_assets = {}
        self.list_of_assets=[]
        for num in range (1,self.nb_USVs+1):
            id = "usv" + str(num)
            self.dict_of_assets[id] = USV(id = id, mission_info = mission_info, 
                                          terrain = terrain, world_model= world_model)
            self.list_of_assets.append(self.dict_of_assets[id])




        