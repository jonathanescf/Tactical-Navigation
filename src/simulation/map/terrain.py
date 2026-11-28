import numpy as np
import matplotlib.pyplot as plt
from src.orchestration.config import parameters
from src.simulation.map.geometry import *


class terrain:
    def __init__(self, bounds:tuple, dict_obs, base_size = (75, 50)):
        self.bounds = bounds
        self.base_size = base_size
        self.spawn = bounds[0] - base_size[0]/2, bounds[3] - base_size[1]/2
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
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)

        x0, x1, y0, y1 = self.bounds
        bw, bh = self.base_size

        in_arena = (x0 < x) & (x < x1) & (y0 < y) & (y < y1)
        in_base = (x0 - bw <= x) & (x <= x0) & (y1 - bh <= y) & (y <= y1)

        return (in_arena | in_base) & (self.distance(x, y) > 0)

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


class discrete_terrain:
    def __init__(self, terrain, resolution):
        self.margin = 10
        self.occupancy_grid = self.build_occupancy_grid(terrain, resolution)   #outside input, we could say it comes from satelite or a aerial drone
        self.origin = self.get_origin(terrain) #coords of top left corner (NW corner). updating when building coordgrid
        self.resolution = resolution #size of one cell in discrete terrain

        

    def get_origin(self, terrain):
        x0 = terrain.bounds[0] - terrain.base_size[0]
        x1, y0, y1 = terrain.bounds[1:] 
        return (x0 - self.margin, y0 - self.margin)


    def build_coord_grid(self,terrain, precision = 0.5):
        """
        creates the uniform gridmap with the coordinates of each cell.
        """
        x0 = terrain.bounds[0] - terrain.base_size[0]
        x1, y0, y1 = terrain.bounds[1:] 
        X = np.arange(x0 - self.margin, x1 + self.margin, precision)
        Y = np.arange(y0 - self.margin, y1 + self.margin, precision)

        map_x, map_y = np.meshgrid(X, Y[::-1]) # That way the gridmad and returns coords with X going right and Y going up
        coordgrid_discrete_terrain = np.stack((map_x, map_y), axis=-1)
        self.origin = (x0 - self.margin, y0 - self.margin)
        return coordgrid_discrete_terrain

    def build_occupancy_grid(self, terrain, precision = 0.5):
        coord_gridmap = self.build_coord_grid(terrain, precision)

        X  =coord_gridmap[..., 0]
        Y  =coord_gridmap[..., 1]

        freemap_bool = np.asarray(terrain.is_free(X, Y), dtype=bool)
        occupancy_gridmap = (~freemap_bool).astype(np.uint8) # inverse of freemap to get obstacle map bool then conversion to int
        return occupancy_gridmap
    