import numpy as np
import matplotlib.pyplot as plt

class map_builder2:
    def __init__(self, grid_positive_length=300):
        self.grid_positive_length = grid_positive_length
        self.grid_length = grid_positive_length * 2
        self.grid_shape = (self.grid_length, self.grid_length)

    def create_map(self):
        cx = self.grid_positive_length
        cy = self.grid_positive_length

        map_array = np.zeros(self.grid_shape)

        for i in range(self.grid_length):
            for j in range(self.grid_length):

                x = i - cx
                y = j - cy

                r = np.sqrt(x**2 + y**2)

                if 50 < r < 100:
                    map_array[i][j] = 1

                if 150 < r < 200:
                    map_array[i][j] = 1

                if 250 < r < 300:
                    map_array[i][j] = 1

                if np.abs(x - y) < 25:
                    map_array[i][j] = 1

        # plt.imshow(map_array, cmap='gray', vmin=0, vmax=1)
        # plt.axis('off')
        # plt.show()


if __name__ == "__main__":
    m = map_builder2()
    m.create_map()

        
