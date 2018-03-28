from udacidrone.connection import Connection
from motion_planning_RRTstar import MotionPlanning_RRTStar
import matplotlib.pyplot as plt
import numpy as np

dummyConn = Connection(False)

def test_main():
    drone = MotionPlanning_RRTStar(dummyConn)

    grid, offsets, wp = drone.planPathImpl()
    mesh = np.meshgrid(
        range(- offsets[1], - offsets[1] + np.size(grid, 0)),
        range(- offsets[0], - offsets[0] + np.size(grid, 1))
    )

    wpArray = np.asarray(wp)
    # plt.imshow(grid, cmap='Greys', origin='lower')
    plt.pcolor(mesh[0], mesh[1], grid)
    plt.plot(wpArray[:, 1], wpArray[:, 0], 'ro-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
