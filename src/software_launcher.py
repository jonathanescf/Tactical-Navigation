import threading
from src.mapbuilder import waypoint
from src.mapbuilder import mapbuilder
from src.simulation import simulation

class SOFTWARE_launcher:

    def __init__(self, parameters):
        self.map_builder = mapbuilder(parameters)
        self.simulation = simulation(parameters, map_builder = self.map_builder)

        t_terminal = threading.Thread(target=self.mission_launcher, daemon=True)
        t_terminal.start()

        self.simulation.run() # blocking function, this is why we need to put the terminal in a thread.

    def mission_launcher(self, mission_to_do=None):
        input("Appuie sur Entrée pour tester les moteurs...")
        for i, mission in self.simulation.missions.items():
            print(f"Bateau {i} → Début de la mission")
            self.simulation.missions[i].go_to_waypoints(self.map_builder.waypoint.dico_of_waypoints)
        input("Appuie sur Entrée pour arrêter la simulation...")
        print("Arrêt de la simulation.")
        

        