
from src.fleet.fleet_manager import fleet_manager
from src.map.mapbuilder import mapbuilder
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from typing import Callable


class graphical_interface:
    def __init__(self, parameters:dict, 
                 fleet_manager:fleet_manager = None, 
                 map_builder:mapbuilder = None,
                 get_static_snapshot:Callable[[], dict] = None,
                 get_dynamic_snapshot:Callable[[float], dict] = None):
        
        self.dt = parameters["dt"]
        self.fleet_manager = fleet_manager
        self.map_builder = map_builder
        self.get_static_snapshot = get_static_snapshot
        self.get_dynamic_snapshot = get_dynamic_snapshot
        print(f'Init Graphical Interface ...')
        print("--------------------")
        self.setup()          

    def setup(self):
        self.setup_display_graphics()
        self.setup_USVs_graphics()
        self.setup_map_graphics()

    def setup_display_graphics(self):
        """
        sets up the window page for the simulation
        """
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_xlim(-300,300)
        self.ax.set_ylim(-300,300)
        self.ax.set_aspect('equal')
        self.ax.set_facecolor("#22272c")
        self.ax.set_title("DDBoat Simulation", color='white', fontsize=12, pad=10)
        self.ax.tick_params(colors='#aaaaaa')
        self.ax.grid(True, linestyle='--', alpha=0.2, color='white')
        self.fig.patch.set_facecolor('#0d1b2a')

    def setup_USVs_graphics(self):
        """
        sets up the USV representation in the simulation
        """
        # flèches représentant chaque bateau
        self.arrows = {}
        list_assets_states = self.get_dynamic_snapshot(0)["assets_state"]
        for asset_usv in list_assets_states:
            # USV = self.fleet_manager.USV[i]
            x, y, psi_rad = asset_usv["pose"]["x"], asset_usv["pose"]["y"], asset_usv["pose"]["psi_rad"]
            self.arrows[asset_usv["id"]] = self.ax.annotate(
                '',
                xy=(x + np.cos(psi_rad) * 4, y + np.sin(psi_rad) * 4),
                xytext=(x,y),
                arrowprops=dict(
                    arrowstyle='->', color='#f5c518',
                    lw=2.5,
                    mutation_scale=20,
                ),
            )

        # légende
        for i in range(1, len(list_assets_states) + 1):
            self.ax.plot([], [], color='#f5c518', label=f'usv asset {i}')
        if len(list_assets_states)  > 0:
            self.ax.legend(loc='upper right', facecolor='#1a2a3a',
                        labelcolor='white', fontsize=8)

    def setup_map_graphics(self):
        """
        sets up the different map elements on the simulation
        """
        if self.map_builder is None:
            return

        self.waypoint_plots  = {}
        self.obstacle_plots = {}

        map_elements = self.get_static_snapshot()

        for waypoint in map_elements["waypoints"]:
            x, y = waypoint["pose"]["x"], waypoint["pose"]["y"]
            plot,  = self.ax.plot(x, y, 'go', markersize=8, zorder=3)
            label  = self.ax.annotate(f"WP{waypoint['id']}", (x, y-3), color='green', fontsize=8)
            self.waypoint_plots[waypoint["id"]] = (plot, label)
        
        for obstacle in map_elements["obstacles"]:
            x, y, radius = obstacle["pose"]["x"], obstacle["pose"]["y"], obstacle["radius"]
            circle = patches.Circle((x, y), radius=radius, fill=True, edgecolor='red', linewidth=2, zorder=3)
            self.ax.add_patch(circle)
            self.obstacle_plots[obstacle["id"]] = circle

    def run(self):
        """
        runs the matplotlib animation
        """
        # animation matplotlib (thread principal)
        self.ani = animation.FuncAnimation(
            self.fig, self.update,
            interval=int(self.dt * 1000),
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()

    def update(self,_): # matplotlib calls this function with a hidden argument frame number.
        """
        updates the graphics
        """
        snap = self.get_dynamic_snapshot(0)  # Update the state of the assets
        self.update_USVs(snap)
        self.update_map(snap)

    def update_USVs(self, snap):
        for asset_usv_state in snap["assets_state"]:
            x, y, psi_rad = asset_usv_state["pose"]["x"], asset_usv_state["pose"]["y"], asset_usv_state["pose"]["psi_rad"]
            queue_fleche = (x, y)
            pointe = (x + np.cos(psi_rad) * 4, y + np.sin(psi_rad) * 4)
            self.arrows[asset_usv_state["id"]].set_position(queue_fleche)
            self.arrows[asset_usv_state["id"]].xy = pointe

    def update_map(self, snap):
        if self.map_builder is None:
            return

        for i, (x, y) in self.map_builder.waypoint.dico_of_waypoints.items():
            plot, label = self.waypoint_plots[i]
            plot.set_data([x], [y])
            label.set_position((x, y))

        for i, (x, y, radius) in self.map_builder.obstacle.dico_of_obstacles.items():
            self.obstacle_plots[i].center = (x, y)
        
        for i in list(self.waypoint_plots.keys()):
            if i not in self.map_builder.waypoint.dico_of_waypoints:
                # waypoint atteint → on efface l'artiste matplotlib
                plot, label = self.waypoint_plots[i]
                plot.remove()
                label.remove()
                del self.waypoint_plots[i]   # on nettoie le dico des artistes
            else:
                plot, label = self.waypoint_plots[i]
                x, y = self.map_builder.waypoint.dico_of_waypoints[i]
                plot.set_data([x], [y])
                label.set_position((x, y))