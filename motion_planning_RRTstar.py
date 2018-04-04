import argparse
import random
import time

import numpy as np
from udacidrone import global_to_local
from udacidrone.connection import MavlinkConnection, Connection

from motion_planning import MotionPlanning, States
from utils import loadGrid
from utils.field import FieldGen
from utils.rrt import RRTStar


class MotionPlanning_RRTStar(MotionPlanning):

    def __init__(self, connection: Connection):
        super().__init__(connection)

        self.goal_local: np.ndarray = None
        self.fieldGen: FieldGen = None

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        # Set self.waypoints
        _, _, wp, _, _, _ = self.planPathImpl()
        wpCanonized = []
        for coord in wp:
            cc = []
            for x in coord:
                cc.append(int(x))
            wpCanonized.append(cc)
        self.waypoints = wpCanonized
        self.send_waypoints()

    def planPathImpl(self,
                     safetyDistance=3,
                     targetAltitude=5
                     ):

        #  read lat0, lon0 from colliders into floating point values
        file = open('colliders.csv', 'r')
        try:
            firstLine = file.readline()
        finally:
            file.close()
        splited = firstLine.strip('\n').split(',')
        lat0, lon0 = map(lambda v: float(v.split(' ')[-1]), splited)

        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)

        # retrieve current global position
        # convert to current local position using global_to_local()
        localPosition = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        grid, east_offset, north_offset = loadGrid(safetyDistance, targetAltitude)
        self.fieldGen = FieldGen(grid, lambda v: v < 0.5)

        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = [int(localPosition[0] + north_offset), int(localPosition[1] + east_offset)]

        # Set goal as some arbitrary position on the grid
        # adapt to set goal as latitude / longitude position and convert
        grid_goal = self.getGoal_grid([north_offset, east_offset])

        print('Local Start and Goal: ', grid_start, grid_goal)

        path, tree = self.getPath(grid_goal=grid_goal, grid_start=grid_start)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # Convert path to waypoints
        waypoints = None
        if path is not None:
            waypoints = [[p[0] - north_offset, p[1] - east_offset, targetAltitude, 0] for p in path]
        offsets = (north_offset, east_offset, 0)
        return grid, offsets, waypoints, tree, grid_start, grid_goal

    def getPath(self, grid_goal, grid_start):
        tree, path = RRTStar(self.fieldGen).run(grid_start, grid_goal)
        return path, tree

    def setGoal_global(self, _global):
        _local = global_to_local(_global, self.global_home)
        self.goal_local = _local

    def setGoal_random(self, offsets):
        grid = self.fieldGen.grid
        _range = list(grid.shape[slice(0, 2)])

        proposal = np.array(list(map(lambda v: random.randint(0, v), _range)))
        self.goal_local = proposal - np.array(offsets)

    def getGoal_grid(self, offsets):
        if self.goal_local is None:
            self.setGoal_random(offsets)
        goal_grid = (self.goal_local + offsets)

        nearestField = self.fieldGen.nearest()
        offsetVec = nearestField[tuple(goal_grid)]
        final = list(goal_grid - offsetVec)
        assert self.fieldGen.grid[final[0], final[1]] == 0
        return final


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning_RRTStar(connection=conn)
    time.sleep(1)

    drone.start()
