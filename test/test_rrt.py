import matplotlib.pyplot as plt
import numpy as np
from udacidrone.connection import Connection

from motion_planning_RRTstar import MotionPlanning_RRTStar

dummyConn = Connection(False)

def _runTest(drone: MotionPlanning_RRTStar):
    # drone = MotionPlanning_RRTStar(dummyConn)

    grid, offsets, wp, tree = drone.planPathImpl()
    mesh = np.meshgrid(
        range(- offsets[1], - offsets[1] + np.size(grid, 0)),
        range(- offsets[0], - offsets[0] + np.size(grid, 1))
    )

    print("start plotting")
    plt.pcolormesh(mesh[0], mesh[1], grid)

    for a, b in tree.edges:
        aa = a.vec - np.array(offsets)[[0,1]]
        bb = b.vec - np.array(offsets)[[0,1]]
        plt.plot([aa[1], bb[1]],
                 [aa[0], bb[0]], 'g-')

    try:
        wpArray = np.asarray(wp)
        plt.plot(wpArray[:, 1], wpArray[:, 0], 'ro-')
    except Exception:
        pass

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def test_RRT():
    _runTest(MotionPlanning_RRTStar(dummyConn))