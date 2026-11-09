import threading
import time
import numpy as np
import matplotlib.pyplot as plt


from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.simulation.simulator import simulator
from src.autonomy.world_model import world_model

class software:

    def __init__(self, parameters:dict, path_scenario):
        self.software_running = True
        self.parameters = parameters

        scenario_info = load_scenario(path_scenario)
        self.terrain = build_terrain(scenario_info)
        self.world_model = world_model(self.terrain)
        self.mission_info = build_mission(scenario_info)
        self.plot_terrain_and_world_model()
 

        # Create instances of the main classes
        self.simulator = simulator(self.parameters, self.mission_info, self.terrain, self.world_model)
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
    
    def plot_terrain_and_world_model(self, path=None, cost=None, n_fine=1000):
        """Terrain (vérité) et costmap (ce que voit le planner) côte à côte, avec le chemin A* optionnel."""
        gm = self.world_model.gridmap              # (H, W, 2), gm[i, j] = (x, y) du centre de la cellule
        res = self.world_model.global_precision
        costmap = self.world_model.costmap

        # Étendue monde de la grille : le coin haut-gauche est gm[0, 0] (y décroît avec i)
        x_left,  y_top    = gm[0, 0]
        x_right, y_bottom = gm[-1, -1]
        extent = [x_left - res/2, x_right + res/2, y_bottom - res/2, y_top + res/2]

        fig, (ax_t, ax_w) = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)

        # --- Gauche : terrain échantillonné plus finement que la grille ---
        xs = np.linspace(extent[0], extent[1], n_fine)
        ys = np.linspace(extent[3], extent[2], n_fine)         # haut -> bas, comme imshow
        truth = np.array([[not self.terrain.is_free(x, y) for x in xs] for y in ys])
        ax_t.imshow(truth, extent=extent, cmap="gray_r", origin="upper")
        ax_t.set_title("Terrain (vérité)")

        # --- Droite : costmap ---
        ax_w.imshow(costmap, extent=extent, cmap="gray_r", origin="upper", vmin=0, vmax=255)
        ax_w.set_title(f"World model (res = {res} m)")

        # --- Chemin A* : indices (i, j) -> coordonnées monde via gridmap ---
        if path:
            rows, cols = zip(*path)
            pts = gm[list(rows), list(cols)]                    # (N, 2)
            for ax in (ax_t, ax_w):
                ax.plot(pts[:, 0], pts[:, 1], "r-o", linewidth=2, markersize=3)
                ax.plot(*pts[0],  "go", markersize=9, label="départ")
                ax.plot(*pts[-1], "b*", markersize=14, label="but")
            ax_w.legend()
            if cost is not None:
                fig.suptitle(f"Coût A* = {cost:.2f}  ({len(path)} cellules)")

        for ax in (ax_t, ax_w):
            ax.set_aspect("equal")
            ax.set_xlabel("x [m]")
        ax_t.set_ylabel("y [m]")
        plt.tight_layout()
        plt.show()
            
