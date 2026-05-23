import threading
from src.mapbuilder import waypoint

class IHM_launcher:
    def __init__(self, map_builder, missions):
        self.map_builder = map_builder
        self.missions = missions

    def common_wrapper(self):
        input("Appuie sur Entrée pour tester les moteurs...")
        for i, mission in self.missions.items():
            print(f"Bateau {i} → Début de la mission")
            mission.go_to_waypoints(self.map_builder.waypoint.dico_of_waypoints)

    def run(self):
        t = threading.Thread(target=self.common_wrapper, daemon=True)
        t.start()
        t.join()
    