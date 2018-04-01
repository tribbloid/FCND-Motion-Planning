import argparse

import matplotlib.pyplot as plt
import numpy as np
from udacidrone.connection import Connection, MavlinkConnection

from motion_planning_P_RRTstar import MotionPlanning_P_RRTStar
from motion_planning_RRTstar import MotionPlanning_RRTStar

dummyConn = Connection(False)


def _runTest(drone: MotionPlanning_RRTStar):
    # drone = MotionPlanning_RRTStar(dummyConn)

    grid, offsets, wp, tree, grid_start, grid_goal = drone.planPathImpl()
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

    start = np.array(grid_start) - np.array(offsets)[[0,1]]
    goal = np.array(grid_goal) - np.array(offsets)[[0,1]]
    start2GoalArray = np.array([start, goal])
    plt.plot(start2GoalArray[:, 1], start2GoalArray[:, 0], 'wo-')
    try:
        wpArray = np.asarray(wp)
        plt.plot(wpArray[:, 1], wpArray[:, 0], 'ro-')
    except Exception:
        pass

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


conn = dummyConn

# parser = argparse.ArgumentParser()
# parser.add_argument('--port', type=int, default=5760, help='Port number')
# parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
# args = parser.parse_known_args()[0]
#
# conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)


def test_RRT():
    _runTest(MotionPlanning_RRTStar(conn))


def test_P_RRT():
    _runTest(MotionPlanning_P_RRTStar(conn))
