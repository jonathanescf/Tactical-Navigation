from src.fleet.USV import USV



class fleet_manager:
    def __init__(self, parameters:dict):
        self.nb_USVs = parameters["nb_of_USVs"]
        self.initialization()
        

    def initialization(self):
        self.USV = {}
        self.list_of_assets=[]
        for id in range (1,self.nb_USVs+1):
            self.USV[id] = USV(id)
            self.list_of_assets.append(self.USV[id])



        