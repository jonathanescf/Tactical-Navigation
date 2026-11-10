import threading
import time
import numpy as np
import matplotlib.pyplot as plt


from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.simulation.simulator import simulator
from src.autonomy.world_model import world_model
from src.autonomy.fleet_supervisor import supervisor

class software:

    def __init__(self, parameters:dict, path_scenario):
        self.software_running = True
        self.parameters = parameters

        scenario_info = load_scenario(path_scenario)
        self.terrain = build_terrain(scenario_info)
        self.world_model = world_model(self.terrain, precision = 5)
        self.mission_info = build_mission(scenario_info)
 

        # Create instances of the main classes
        self.simulator = simulator(self.parameters, self.mission_info, self.terrain)
        self.supervisor = supervisor(self.world_model, self.mission_info, self.simulator.dict_of_assets)

        self.graphical_interface = graphical_interface(self.parameters, 
                                                       get_static_snapshot = self.static_snapshot,
                                                       get_dynamic_snapshot = self.dynamic_snapshot)
        
        # Start everything in separate threads
        self.thread_mission_loop = threading.Thread(target=self.loop, daemon=True)
        self.thread_mission_loop.start()

        self.graphical_interface.run()

    def loop(self):
        while self.software_running:
            for asset in self.simulator.dict_of_assets.values():
                asset.physic_step(self.parameters["dt"])
            self.supervisor.fulfill_mission()
            time.sleep(self.parameters["dt"])

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
            if asset.do.ongoing_target_state is None:
                ongoing = None
            else:
                ongoing = {"x": asset.do.ongoing_target_state[0], 
                           "y": asset.do.ongoing_target_state[1]}
            list_assets_states.append({"id": asset.id,
                                       "type": asset.type,
                                       "pose": {"x": asset.x,
                                                "y": asset.y,
                                                "psi_rad": asset.psi_rad},
                                        "ongoing_target": ongoing})

        return {"time": t_s,
                "dt" : self.parameters["dt"], 
                "assets_state": list_assets_states}
