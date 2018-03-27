import numpy as np

from utils.field import Field2DGen, plot

def test_buildField():
    grid = np.ones((20, 30), np.int)
    grid[6,5:8] = 0

    gen = Field2DGen(grid)
    field = gen.buildToClosestBoundary()

    # plt.imshow(grid, cmap='Greys', origin='lower')
    plot(field)
