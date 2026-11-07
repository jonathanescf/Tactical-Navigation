import threading
import time

from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.simulation.simulator import simulator

class software:

    def __init__(self, parameters:dict, path_scenario):
        self.software_running = True
        self.parameters = parameters

        scenario_info = load_scenario(path_scenario)
        self.terrain = build_terrain(scenario_info)
        self.mission_info = build_mission(scenario_info)
 

        # Create instances of the main classes
        self.simulator = simulator(self.parameters, self.mission_info, self.terrain)
        self.graphical_interface = graphical_interface(self.parameters, 
                                                       get_static_snapshot = self.static_snapshot,
                                                       get_dynamic_snapshot = self.dynamic_snapshot)
        
        # Start everything in separate threads
        self.initialize_threads()
        self.graphical_interface.run()
        
    def physics_loop(self):
        while self.software_running:
            for asset in self.simulator.dict_of_assets.values():
                asset.physic_step(self.parameters["dt"])
            time.sleep(self.parameters["dt"])

    def brain_loop(self):
        while self.software_running:
            for asset in self.simulator.list_of_assets:
                asset.do.brain_step()
            time.sleep(self.parameters["dt"])

    def initialize_USVs(self):
        
        self.thread_brain = threading.Thread(target=self.brain_loop, daemon=True)
        self.thread_brain.start()
    
    def initialize_threads(self):
        
        self.thread_init_usvs = threading.Thread(target=self.initialize_USVs, daemon=True)
        self.thread_init_usvs.start()

        self.thread_physics_loop = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread_physics_loop.start()

    def static_snapshot(self):
        return {
            "bounds": self.terrain.bounds,
            "base_size": self.terrain.base_size,
            "waypoints": [{"id": i, "pose": {"x": x, "y": y}}
                        for i, (x, y) in enumerate(self.mission_info["waypoints"], start=1)],
            "terrain_distance": self.terrain.distance,  # a modifier lorsque le gui sera sur web 
                                                        # (impossible de serialiser une fonction comme ca)
        }

    def dynamic_snapshot(self, t_s):
        """
        Creates a dict with the state of every asset at the instant it is called. Useful to record simulation logs and for visualization.
        """
        list_assets_states = []
        for asset in self.simulator.list_of_assets:
            list_assets_states.append({"id": asset.id,
                                       "type": asset.type,
                                       "pose": {"x": asset.x,
                                                "y": asset.y,
                                                "psi_rad": asset.psi_rad},
                                        "ongoing_target": {"x": asset.do.ongoing_target_state[0], 
                                                           "y": asset.do.ongoing_target_state[1]}})

        return {"time": t_s,
                "dt" : self.parameters["dt"], 
                "assets_state": list_assets_states}
    
    #TODO: should stop every thread with a stop event.

        
