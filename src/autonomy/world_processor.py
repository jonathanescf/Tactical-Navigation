import cv2
import numpy as np
import matplotlib.pyplot as plt

class process_world: #TODO! finish
    def __init__(self, discret_terrain_occupancy_grid, discrete_terrain_res, discrete_terrain_origin, low_res, inflation_length, decay_length):
        self.discrete_terrain_occupancy = discret_terrain_occupancy_grid   #outside input, we could say it comes from satelite or a aerial drone
        self.discrete_terrain_origin = discrete_terrain_origin #coords of top left corner (NW corner)
        self.discrete_terrain_res = discrete_terrain_res #size of one cell in discrete terrain

        # Eahc of these following maps is used for the logic in autonomy, it is of lower resolution
        self.low_res = low_res # size of one cell in the computed gridmaps
        self.occupancy_gridmap = self.build_occupancy_gridmap()
        self.coord_gridmap = self.build_coord_gridmap()
        self.cost_gridmap = self.build_cost_gridmap(inflation_length, decay_length)

    #TODO! understand the functions !!!!!!!!!!
    def build_occupancy_gridmap(self):

        factor = round(self.low_res / self.discrete_terrain_res)

        h, w = self.discrete_terrain_occupancy.shape
        pad_h = (-h) % factor
        pad_w = (-w) % factor
        padded = np.pad(self.discrete_terrain_occupancy, ((0, pad_h), (0, pad_w)),
                        mode="constant", constant_values=0)

        H, W = padded.shape
        blocks = padded.reshape(H // factor, factor, W // factor, factor)
        return blocks.max(axis=(1, 3))

    def build_coord_gridmap(self):
        """array (H, W, 2) : coord_gridmap[i, j] = (x, y) the coords of cell center (i, j)."""
        H, W = self.occupancy_gridmap.shape
        x0, y0 = self.discrete_terrain_origin

        xs = x0 + (np.arange(W) + 0.5) * self.low_res   
        ys = y0 - (np.arange(H) + 0.5) * self.low_res   

        X, Y = np.meshgrid(xs, ys)                      
        return np.stack((X, Y), axis=-1)

    def build_cost_gridmap(self, inflation_length, affected_distance):
        ### inflating the obstacles by dilating the image.

        r = int(np.ceil(inflation_length / self.low_res))

        yy, xx = np.ogrid[-r:r + 1, -r:r + 1] #
        kernel = (xx**2 + yy**2 <= r**2).astype(np.uint8)

        inflated_map = cv2.dilate(self.occupancy_gridmap, kernel)

        ### Putting an exponential decrease cost

        occ = inflated_map.astype(bool)
        free = (~occ).astype(np.uint8)    # array: 0 is obstacle one is free

        dist = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) # computes nb of cell distance from 0 value cells
        dist = dist * self.low_res              # cell dist unit -> meter unit

        k = np.log(255) / affected_distance
        cost = 255 * np.exp(-k * dist)

        cost[occ] = 255

        return np.rint(np.clip(cost, 0, 255)).astype(np.uint8)

    def get_ind_from_coord(self, x, y):
        i_max, j_max, _ = np.shape(self.coord_gridmap)
        print("x = ", x)
        print("y = ", y)
        x0, y0 = self.coord_gridmap[-1, 0]
        print("x0 = ", x0)
        print("y0 = ", y0)
        j = int(round((x - x0) / self.low_res))
        k = int(round((y - y0) / self.low_res))
        i = i_max - 1 - k # Y axis inverted wr to i (Y[::-1])
        print("i = ", i)
        print("j = ", j)
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

    # ---------- Terrain discrétisé (haute résolution) ----------
    terrain_res = 0.1                      # m / cellule
    size_m = 500.0                         # carte de 500 m x 500 m
    n = int(size_m / terrain_res)          # 1000 x 1000 cellules
    origin = (0.0, size_m)                 # coin NW (x0, y0), y vers le nord

    xs = origin[0] + (np.arange(n) + 0.5) * terrain_res
    ys = origin[1] - (np.arange(n) + 0.5) * terrain_res
    X = xs[None, :]                        # broadcasting : évite un meshgrid complet
    Y = ys[:, None]

    terrain = np.zeros((n, n), dtype=np.uint8)                       # 0 = libre, 1 = obstacle

    # Obstacles fixes
    terrain[(X > 100) & (X < 115) & (Y > 100) & (Y < 400)] = 1       # mur vertical
    terrain[(X > 250) & (X < 450) & (Y > 330) & (Y < 345)] = 1       # mur horizontal
    terrain[(X - 350) ** 2 + (Y - 150) ** 2 < 50.0 ** 2] = 1         # grande île
    terrain[(X - 200) ** 2 + (Y - 220) ** 2 < 20.0 ** 2] = 1         # île moyenne

    # Rochers aléatoires (graine fixe pour un résultat reproductible)
    rng = np.random.default_rng(0)
    for _ in range(40):
        cx, cy = rng.uniform(20, size_m - 20, size=2)
        radius = rng.uniform(3, 10)
        terrain[(X - cx) ** 2 + (Y - cy) ** 2 < radius ** 2] = 1

    # ---------- Modèle de monde ----------
    world = process_world(
        terrain, terrain_res, origin,
        low_res=2.0,              # multiple de terrain_res (2.0 / 0.5 = 4)
        inflation_length=5.0,     # rayon létal autour des obstacles (m)
        decay_length=30.0,        # distance à partir de laquelle le coût passe sous 1 (m)
    )

    print("terrain       :", terrain.shape)
    print("occupancy low :", world.occupancy_gridmap.shape)
    print("coord_gridmap :", world.coord_gridmap.shape,
          "| centre cellule (0,0) =", world.coord_gridmap[0, 0])
    print("costmap       :", world.cost_gridmap.shape, world.cost_gridmap.dtype)

    # ---------- Affichage côte à côte ----------
    def extent(shape, res, origin):
        H, W = shape
        x0, y0 = origin
        return [x0, x0 + W * res, y0 - H * res, y0]   # [gauche, droite, bas, haut]

    fig, axs = plt.subplots(1, 3, figsize=(21, 7))

    axs[0].imshow(terrain, extent=extent(terrain.shape, terrain_res, origin),
                  cmap="gray_r", origin="upper", vmin=0, vmax=1)
    axs[0].set_title(f"Terrain discrétisé (res = {terrain_res} m)")

    axs[1].imshow(world.occupancy_gridmap,
                  extent=extent(world.occupancy_gridmap.shape, world.low_res, origin),
                  cmap="gray_r", origin="upper", vmin=0, vmax=1)
    axs[1].set_title(f"Occupancy low-res (res = {world.low_res} m)")

    im = axs[2].imshow(world.cost_gridmap,
                       extent=extent(world.cost_gridmap.shape, world.low_res, origin),
                       cmap="gray_r", origin="upper", vmin=0, vmax=255)
    axs[2].set_title("Costmap [0-255]")
    fig.colorbar(im, ax=axs[2], fraction=0.046, pad=0.04)

    for ax in axs:
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

    plt.tight_layout()
    plt.show()