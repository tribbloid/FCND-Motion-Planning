import matplotlib.pyplot as plt
import numpy as np

from utils import loadGrid
from utils.field import FieldGen


def test_buildField():

    # grid = np.zeros((20, 30), np.int)
    # grid[6,5:8] = 1

    grid, _, _ = loadGrid()

    gen = FieldGen(grid)
    field = gen.nearest()

    # plt.imshow(grid, cmap='Greys', origin='lower')
    # quiver(field)
    plt.subplot(121)
    normColor(field)
    plt.subplot(122)
    normColor(gen.coverage)
    plt.show()


def quiver(field=None, points=None):
    if field is not None:
        plt.quiver(field[:, :, 1], field[:, :, 0])
    if points is not None:
        x = list(map(lambda v: v[1], points))
        y = list(map(lambda v: v[0], points))
        plt.scatter(x, y, s=20)
    plt.xlabel('X')
    plt.ylabel('Y')


def normColor(field=None, points=None):
    if field is not None:
        norm = np.linalg.norm(field, axis=2)
        plt.pcolormesh(norm)
    if points is not None:
        x = list(map(lambda v: v[1], points))
        y = list(map(lambda v: v[0], points))
        plt.scatter(x, y, s=20)
    plt.xlabel('X')
    plt.ylabel('Y')
