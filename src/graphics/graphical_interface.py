
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from typing import Callable


class graphical_interface:
    def __init__(self, parameters:dict, 
                 get_static_snapshot:Callable[[], dict] = None,
                 get_dynamic_snapshot:Callable[[float], dict] = None):
        
        self.dt = parameters["dt"]
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
        Sets up the static map elements (drawn once) and the containers
        for the dynamic ones (created on demand in update_map).
        """
        self.obstacle_plots = {}
        self.target_plots = {}

        map_elements = self.get_static_snapshot()

        for obstacle in map_elements["obstacles"]:
            x, y, radius = obstacle["pose"]["x"], obstacle["pose"]["y"], obstacle["radius"]
            circle = patches.Circle((x, y), radius=radius, fill=True,
                                    edgecolor='red', linewidth=2, zorder=3)
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
        for a in snap["assets_state"]:
            t = a.get("ongoing_target")
            entry = self.target_plots.get(a["id"])

            if t is None:
                if entry:
                    entry[0].set_data([], [])
                    entry[1].set_visible(False)
                continue

            if entry is None:   # création à la volée, une seule fois par asset
                plot, = self.ax.plot([], [], 'g*', markersize=14, zorder=3)
                label = self.ax.annotate('', (0, 0), color='green', fontsize=8)
                entry = self.target_plots[a["id"]] = (plot, label)

            plot, label = entry
            plot.set_data([t["x"]], [t["y"]])
            label.set_text(f"TARGET for USV {a['id']}")
            label.set_position((t["x"], t["y"] - 3))
            label.set_visible(True)