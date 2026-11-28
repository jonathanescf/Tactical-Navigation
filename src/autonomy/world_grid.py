import math

import numpy as np
import matplotlib.pyplot as plt

class process_world: #TODO! finish
    def __init__(self, discret_terrain_occupancy_grid, discrete_terrain_res, low_res):
        self.occupancy_gridmap = discret_terrain_occupancy_grid   # outside input, we could say it comes from satelite or a aerial drone
        self.high_res = discrete_terrain_res

        self.low_res = low_res
        self.occupancy_gridmap = self.build_occupancy_gridmap()
        self.coord_gridmap = self.build_coord_gridmap()
        self.cost_gridmap = self.build_costmap() 

    #TODO! understand the functions !!!!!!!!!!


    def build_occupancy_gridmap(self):

        factor = round(self.light_gridmap_precision / self.gridmap_precision)

        h, w = self.gridmap.shape
        pad_h = (-h) % factor
        pad_w = (-w) % factor
        padded = np.pad(self.gridmap, ((0, pad_h), (0, pad_w)),
                        mode="constant", constant_values=0)

        H, W = padded.shape
        blocks = padded.reshape(H // factor, factor, W // factor, factor)
        return blocks.max(axis=(1, 3))

    def build_coord_gridmap(self):
        pass

    def build_costmap(self):
        """Costmap uint8 construite uniquement depuis gridmap et boat_length.
        254 : létal (obstacle ou à moins de boat_length/2 d'un obstacle)
        253 -> 0 : décroissance exponentielle jusqu'à inflation_radius
        0   : libre et loin de tout obstacle"""
        p = self.gridmap_precision
        self.lethal_radius = 2
        self.inflation_radius = 4
        self.decay = 0.01
        occupied = (self.gridmap > 0).astype(np.uint8)
        h, w = occupied.shape
        cost = np.zeros((h, w), dtype=np.uint8)

        n = int(np.ceil(self.inflation_radius / p))  # rayon d'inflation en cellules

        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                d = np.hypot(dy, dx) * p
                if d > self.inflation_radius:
                    continue

                if d <= self.lethal_radius:
                    c = 254
                else:
                    c = int(253 * np.exp(-self.decay * (d - self.lethal_radius)))
                if c == 0:
                    continue

                # Une cellule occupée en (i, j) impacte la cellule (i+dy, j+dx)
                src = occupied[max(0, -dy):h - max(0, dy),
                               max(0, -dx):w - max(0, dx)]
                dst = cost[max(0, dy):h + min(0, dy),
                           max(0, dx):w + min(0, dx)]   # vue sur cost
                np.maximum(dst, src * np.uint8(c), out=dst)

        return cost

    def get_ind_from_coord_in_coord_gridmap(self, x, y):
        i_max, j_max, _ = np.shape(self.gridmap)
        x0, y0 = self.gridmap[-1, 0]
        j = int(round((x - x0) / self.gridmap_precision))
        k = int(round((y - y0) / self.gridmap_precision))
        i = i_max - 1 - k # Y axis inverted wr to i (Y[::-1])
        if 0 <= i < i_max and 0 <= j < j_max:
            return i, j
        return None

    def get_ind_from_coord_in_light_coord_gridmap(self, x, y):
        i_max, j_max, _ = np.shape(self.light_gridmap)
        x0, y0 = self.gridmap[-1, 0]
        j = int(round((x - x0) / self.light_gridmap_precision))
        k = int(round((y - y0) / self.light_gridmap_precision))
        i = i_max - 1 - k # Y axis inverted wr to i (Y[::-1])
        if 0 <= i < i_max and 0 <= j < j_max:
            return i, j
        return None

    def plot_world_grid(self, path=None):
        gm = self.gridmap
        res = self.gridmap_precision
        costmap = self.costmap

        x_left,  y_top    = gm[0, 0]
        x_right, y_bottom = gm[-1, -1]
        extent = [x_left - res/2, x_right + res/2, y_bottom - res/2, y_top + res/2]

        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(costmap, extent=extent, cmap="gray_r", origin="upper", vmin=0, vmax=255)
        ax.set_title(f"World model (res = {res} m)")

        if path is not None and len(path) > 0:
            pts = np.asarray(path)          # (N, 2) en mètres
            ax.plot(pts[:, 0], pts[:, 1], "r-o", linewidth=2, markersize=3)
            ax.plot(*pts[0],  "go", markersize=9, label="départ")
            ax.plot(*pts[-1], "b*", markersize=14, label="but")

        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.tight_layout()
        plt.show()





class world_grid: #obsolete
    def __init__(self, terrain, precision = 1):
        self.global_precision = precision
        self.gridmap = self.build_gridmap(terrain)
        self.costmap = self.build_costmap(terrain)



    def build_costmap(self, terrain, affected_distance=5): # affected distance is distance from which the cost is less the 1
        X = self.gridmap[..., 0] # ... is the ellipsis operator, it means that we dont care about the shape of X, we just take the first component of each value inside.
        Y = self.gridmap[..., 1]
        D = terrain.distance(X, Y)                       
        k = np.log(255) / affected_distance
        return np.minimum(255, 255 * np.exp(-k * D))




        


if __name__ == "__main__":
    # --- Paramètres du test ---
    precision = 0.5          # m / cellule (gridmap)
    light_precision = 2.0    # m / cellule (light gridmap) -> facteur 4
    boat_length = 4.0        # m
    H = W = 200              # 100 m x 100 m

    # --- Gridmap synthétique : 0 = libre, 255 = occupé (comme une image) ---
    raw = np.zeros((H, W), dtype=np.uint8)
    raw[0, :] = raw[-1, :] = 255                      # bords
    raw[:, 0] = raw[:, -1] = 255
    raw[60:80, 40:140] = 255                          # mur horizontal
    raw[120:160, 100:110] = 255                       # barre verticale
    yy, xx = np.ogrid[:H, :W]
    raw[(yy - 150) ** 2 + (xx - 50) ** 2 <= 15 ** 2] = 255   # île circulaire

    # --- Clip : on ramène la gridmap en binaire 0/1 ---
    gridmap = np.clip(raw, 0, 1)

    world = process_world(gridmap, precision, light_precision)

    # --- Affichage ---
    def show(ax, grid, res, title):
        h, w = grid.shape
        ax.imshow(grid, extent=[0, w * res, 0, h * res], origin="upper",
                  cmap="gray_r", vmin=0.25, vmax=1)
        ax.set_title(f"{title}\nshape={grid.shape}, res={res} m")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    show(axes[0], world.gridmap, world.gridmap_precision, "Gridmap")
    show(axes[1], world.light_gridmap, world.light_gridmap_precision, "Light gridmap")
    show(axes[2], world.costmap, world.gridmap_precision,
         f"Costmap binaire (rayon = {boat_length / 2} m)")
    plt.tight_layout()
    plt.show()

        
