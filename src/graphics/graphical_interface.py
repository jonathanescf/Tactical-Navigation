
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches


class graphical_interface:
    def __init__(self, parameters, fleet_manager = None, map_builder = None):
        self.dt = parameters["dt"]
        self.fleet_manager = fleet_manager
        self.map_builder = map_builder
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
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
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
        for i in range(1, self.fleet_manager.nb_USVs + 1):
            b = self.fleet_manager.USV[i]
            self.arrows[i] = self.ax.annotate(
                '',
                xy=(b.x + np.cos(b.psi_rad) * 4, b.y + np.sin(b.psi_rad) * 4),
                xytext=(b.x, b.y),
                arrowprops=dict(
                    arrowstyle='->', color='#f5c518',
                    lw=2.5,
                    mutation_scale=20,
                ),
            )

        # légende
        for i in range(1, self.fleet_manager.nb_USVs + 1):
            self.ax.plot([], [], color='#f5c518', label=f'Bateau {i}')
        if self.fleet_manager.nb_USVs > 0:
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

        for i, (x, y) in self.map_builder.waypoint.dico_of_waypoints.items():
            plot,  = self.ax.plot(x, y, 'go', markersize=8, zorder=3)
            label  = self.ax.annotate(f"WP{i}", (x, y-3), color='green', fontsize=8)
            self.waypoint_plots[i] = (plot, label)
        
        for i, (x, y, radius) in self.map_builder.obstacle.dico_of_obstacles.items():
            circle = patches.Circle((x, y), radius=radius, fill=True, edgecolor='red', linewidth=2, zorder=3)
            self.ax.add_patch(circle)
            self.obstacle_plots[i] = circle

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
        self.update_USVs()
        self.update_map()

    def update_USVs(self):
        for i in range(1, self.fleet_manager.nb_USVs + 1):
            USV = self.fleet_manager.USV[i]
            USV.step(self.dt)

            queue_fleche = (USV.x, USV.y)
            pointe = (USV.x + np.cos(USV.psi_rad) * 4, USV.y + np.sin(USV.psi_rad) * 4)
            self.arrows[i].set_position(queue_fleche)
            self.arrows[i].xy = pointe

    def update_map(self):
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