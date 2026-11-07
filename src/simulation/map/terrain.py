import numpy as np
import matplotlib.pyplot as plt
from src.orchestration.config import parameters
from src.simulation.map.geometry import *


class terrain:
    def __init__(self, bounds:tuple, dict_obs, base_size = (75, 50)):
        self.bounds = bounds
        self.base_size = base_size
        self.dx = parameters["dx"]
        self.dy = parameters["dy"]
        self.init_field(dict_obs)

    def init_field(self, dict_obs):
        x0, x1, y0, y1 = self.bounds
        free_space = polygon([(x0, y0), (x1, y0), (x1, y1),
                           (x0 - self.base_size[0], y1),          
                           (x0 - self.base_size[0], y1 - self.base_size[1]),
                           (x0, y1 - self.base_size[1])
                           ])
        
        self.walls = lambda x, y: -free_space(x, y)

        self.field=union(*([circle(*o) for o in dict_obs["circle"]] + 
                           [rectangle(*o) for o in dict_obs["rectangle"]] +
                           [self.walls]))

    def distance(self, x, y):
        return self.field(x,y)

    def is_free(self, x, y):
        is_inside = (self.bounds[0] < x < self.bounds[1] and 
                     self.bounds[2] < y < self.bounds[3])
        return is_inside and self.distance(x, y) > 0

    def gradient(self, x, y):
        """gradient normalized: opposite direction of closest obstacle"""
        gx = (self.distance(x + self.dx, y) - self.distance(x - self.dx, y)) / (2 * self.dx)
        gy = (self.distance(x, y + self.dy) - self.distance(x, y - self.dy)) / (2 * self.dy)
        gx_normalized = gx / np.linalg.norm([gx, gy]) 
        gy_normalized = gy / np.linalg.norm([gx, gy]) 
        return gx_normalized, gy_normalized

    def plot_terrain(self):
        x0, x1, y0, y1 = self.bounds
        x0 -= 1.1 * max(self.base_size)

        xs = np.linspace(x0, x1, 400)
        ys = np.linspace(y0, y1, 400)
        XX, YY = np.meshgrid(xs, ys)
        D = self.distance(XX, YY)

        fig, ax = plt.subplots(figsize=(7, 7))

        im = ax.imshow(D, extent=(x0, x1, y0, y1), origin="lower", 
                       cmap="RdYlGn", vmin =0, vmax=20)
        ax.contour(XX, YY, D, levels=[0], colors="black", linewidths=1.5)
        fig.colorbar(im, ax=ax, label="Potentiel")

        ax.set_aspect("equal")
        ax.set_title("Potentiel du terrain")
        plt.show()




    