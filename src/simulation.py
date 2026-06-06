from src.physics import ddboat_state

from fake_boat_drivers.fake_boat_driver_arduino import fake_boat_ardu
from fake_boat_drivers.fake_boat_driver_gps import fake_boat_gps
from fake_boat_drivers.fake_boat_driver_imu import fake_boat_imu

from src.missions import ddboat_mission

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches



class simulation:
    def __init__(self, parameters, map_builder=None):
        self.nb_boats = parameters["nb_of_boats"]
        self.map_builder = map_builder
        self.dt = parameters["dt"]
        self.initialization()

    def initialization(self):   
        """
        Creates all the attributes for the class.

        the key of each dict represents the id of the boat, the value is the class instance for drivers state and missions..
        """
        self.boats     = {}
        self.fake_ardu = {}
        self.fake_gps  = {}
        self.fake_imu  = {}
        self.missions  = {}
        for i in range (1,self.nb_boats+1):
            self.boats[i]     = ddboat_state(id=int(i))
            self.fake_ardu[i] = fake_boat_ardu(self.boats[i])
            self.fake_gps[i]  = fake_boat_gps(self.boats[i])
            self.fake_imu[i]  = fake_boat_imu(self.boats[i])
            self.missions[i]  = ddboat_mission(waypoint=self.map_builder.waypoint, ardu = self.fake_ardu[i],gps = self.fake_gps[i],imu = self.fake_imu[i])    

        self.setup()  

    def setup(self):
        self.setup_display_graphics()
        self.setup_boats_graphics()
        self.setup_map_graphics()

    def setup_display_graphics(self):
        """
        sets up the window page for the simulation
        """
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_aspect('equal')
        self.ax.set_facecolor('#0d1b2a')
        self.ax.set_title("DDBoat Simulation", color='white', fontsize=12, pad=10)
        self.ax.tick_params(colors='#aaaaaa')
        self.ax.grid(True, linestyle='--', alpha=0.2, color='white')
        self.fig.patch.set_facecolor('#0d1b2a')

    def setup_boats_graphics(self):
        """
        sets up the boat representation in the simulation
        """
        # flèches représentant chaque bateau
        self.arrows = {}
        for i in range(1, self.nb_boats + 1):
            b = self.boats[i]
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
        for i in range(1, self.nb_boats + 1):
            self.ax.plot([], [], color='#f5c518', label=f'Bateau {i}')
        if self.nb_boats > 0:
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
        self.update_boats()
        self.update_map()

    def update_boats(self):
        for i in range(1, self.nb_boats + 1):
            boat = self.boats[i]
            boat.step(self.dt)

            queue_fleche = (boat.x, boat.y)
            pointe = (boat.x + np.cos(boat.psi_rad) * 4, boat.y + np.sin(boat.psi_rad) * 4)
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