import numpy as np
import matplotlib.pyplot as plt
from src.simulation.map.terrain import terrain



class world_model:
    def __init__(self, terrain, precision = 0.5):
        self.global_precision = precision
        self.gridmap = self.build_gridmap(terrain)
        self.costmap = self.build_costmap(terrain)

    def build_gridmap(self,terrain):
        """
        creates the uniform gridmap with the coordinates of each cell.
        """
        x0 = terrain.bounds[0] - terrain.base_size[0]
        x1, y0, y1 = terrain.bounds[1:]
        X = np.arange(x0, x1, self.global_precision)
        Y = np.arange(y0, y1, self.global_precision)

        map_x, map_y = np.meshgrid(X, Y[::-1]) # That way the gridmad and returns coords with X going right and Y going up
        gridmap = np.stack((map_x, map_y), axis=-1)
        return gridmap

    def build_costmap(self, terrain):
        i_max, j_max, _ = np.shape(self.gridmap)
        costmap=np.zeros((i_max, j_max))
        for i in range(i_max):
            for j in range(j_max):
                x, y = self.gridmap[i,j]
                if not terrain.is_free(x, y):
                    costmap[i,j]= 255
        return costmap

    def get_ind_from_coord(self, x, y):
        i_max, j_max, _ = np.shape(self.gridmap)
        x0, y0 = self.gridmap[-1, 0]
        j = int(round((x - x0) / self.global_precision))
        k = int(round((y - y0) / self.global_precision))
        i = i_max - 1 - k # Y axis inverted wr to i (Y[::-1])
        if 0 <= i < i_max and 0 <= j < j_max:
            return i, j
        return None

    def plot_world_model(self, path=None, cost=None):
        gm = self.gridmap
        res = self.global_precision
        costmap = self.costmap

        x_left,  y_top    = gm[0, 0]
        x_right, y_bottom = gm[-1, -1]
        extent = [x_left - res/2, x_right + res/2, y_bottom - res/2, y_top + res/2]

        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(costmap, extent=extent, cmap="gray_r", origin="upper", vmin=0, vmax=255)
        ax.set_title(f"World model (res = {res} m)")

        if path:
            pts = gm[[i for i, _ in path], [j for _, j in path]]   # (N, 2)
            ax.plot(pts[:, 0], pts[:, 1], "r-o", linewidth=2, markersize=3)
            ax.plot(*pts[0],  "go", markersize=9, label="départ")
            ax.plot(*pts[-1], "b*", markersize=14, label="but")
            ax.legend()
            if cost is not None:
                ax.set_title(f"World model (res = {res} m), coût A* = {cost:.2f} ({len(path)} cellules)")

        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.tight_layout()
        plt.show()
        


if __name__ == "__main__":

    # Fake terrain for testing
    class Terrain:
        bounds = (-1, 1, 0, 3)
        base_size = (1, 1)

        def is_free(self, x, y):
            return True

    terrain = Terrain()

    # Create world model
    world = world_model(precision=1.0, terrain=terrain)

    gridmap = world.gridmap

    n_rows, n_cols, _ = gridmap.shape

    print("Gridmap shape:", gridmap.shape)

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))

    # Display grid
    ax.imshow(
        np.zeros((n_rows, n_cols)),
        extent=[
            gridmap[0, 0, 0] - 0.5,
            gridmap[0, -1, 0] + 0.5,
            gridmap[-1, 0, 1] - 0.5,
            gridmap[0, 0, 1] + 0.5
        ],
        cmap="Greys",
        alpha=0.15
    )

    # Draw grid lines
    for i in range(n_rows + 1):
        ax.axhline(
            gridmap[0, 0, 1] + 0.5 - i,
            color="black",
            linewidth=0.8
        )

    for j in range(n_cols + 1):
        ax.axvline(
            gridmap[0, 0, 0] - 0.5 + j,
            color="black",
            linewidth=0.8
        )

    # Display coordinates and indices in each cell
    for i in range(n_rows):
        for j in range(n_cols):

            x = gridmap[i, j, 0]
            y = gridmap[i, j, 1]

            label = f"[{i},{j}]\n({x:.1f}, {y:.1f})"

            ax.text(
                x, y,
                label,
                ha="center",
                va="center",
                fontsize=9
            )

            # Print coordinates in terminal
            print(f"Cell [{i},{j}] -> X = {x}, Y = {y}")

    # Configure plot
    ax.set_title("Gridmap - Cell indices and coordinates")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.set_xticks(np.arange(
        gridmap[0, 0, 0] - 0.5,
        gridmap[0, -1, 0] + 1,
        1
    ))

    ax.set_yticks(np.arange(
        gridmap[-1, 0, 1] - 0.5,
        gridmap[0, 0, 1] + 1,
        1
    ))

    ax.set_aspect("equal")
    ax.grid(False)

    plt.show()







        
