import argparse
import time

from udacidrone.connection import MavlinkConnection

from motion_planning_RRTstar import MotionPlanning_RRTStar
from utils.rrt import P_RRTStar


class MotionPlanning_P_RRTStar(MotionPlanning_RRTStar):

    @staticmethod
    def getPath(fieldGen, grid_goal, grid_start):
        tree, path = P_RRTStar(fieldGen).run(grid_start, grid_goal)
        return path, tree


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning_P_RRTStar(conn)
    time.sleep(1)

    drone.start()
