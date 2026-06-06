import threading
from src.mapbuilder import mapbuilder
from src.graphical_interface import graphical_interface
from src.fleet_manager import fleet_manager

class SOFTWARE_launcher:

    def __init__(self, parameters):
        # Create instances of the main classes
        self.fleet_manager = fleet_manager(parameters)
        self.map_builder = mapbuilder(parameters)
        self.graphical_interface = graphical_interface(parameters, fleet_manager = self.fleet_manager, map_builder = self.map_builder)
        
        # Start the mission launcher in a separate thread
        t_terminal = threading.Thread(target=self.mission_launcher, daemon=True)
        t_terminal.start()
        self.graphical_interface.run() # blocking function, this is why we need to put the terminal in a thread.

    def mission_launcher(self, mission_to_do=None):
        input("Appuie sur Entrée pour tester les moteurs...")
        id = 1 # we only have one boat for now. This should be changed when we will have more boats.
        
        print(f"Bateau {id} → Début de la mission")
        self.fleet_manager.boat[id].do.test()
        input("Appuie sur Entrée pour arrêter la simulation...")
        print("Arrêt de la simulation.")
        

        