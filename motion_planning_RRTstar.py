import argparse
import random
import time

import numpy as np
from udacidrone import global_to_local
from udacidrone.connection import MavlinkConnection

from motion_planning import MotionPlanning, States
from planning_utils import a_star, heuristic, create_grid
from utils.RRTStar import RRTStar


class MotionPlanning_RRTStar(MotionPlanning):

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        # Set self.waypoints
        _, wp = self.planPathImpl()
        self.waypoints = wp
        print(self.waypoints)
        # TODO: send waypoints to sim
        self.send_waypoints()

    def planPathImpl(self,
                     safetyDistance=5,
                     targetAltitude=3
                     ):

        #  read lat0, lon0 from colliders into floating point values
        file = open('colliders.csv','r')
        try:
            firstLine = file.readline()
        finally:
            file.close()
        splitted = firstLine.strip('\n').split(',')
        lat0, lon0 = map(lambda v: v.split(' ')[-1], splitted)

        # set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)

        # retrieve current global position
        # convert to current local position using global_to_local()
        localPosition = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        # Determine offsets between grid and map
        north_offset = int(np.abs(np.min(data[:, 0])))
        east_offset = int(np.abs(np.min(data[:, 1])))
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define a grid for a particular altitude and safety margin around obstacles
        grid = create_grid(data, targetAltitude, safetyDistance)

        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(localPosition[0]+north_offset), int(localPosition[1]+east_offset))

        # Set goal as some arbitrary position on the grid
        # adapt to set goal as latitude / longitude position and convert
        grid_goal = self.getRandomGoal(grid)

        print('Local Start and Goal: ', grid_start, grid_goal)
        _, path = RRTStar(grid).run(grid_start, grid_goal, 0.5, 10)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # Convert path to waypoints
        waypoints = [[p[0] - north_offset, p[1] - east_offset, targetAltitude, 0] for p in path]
        offsets = (north_offset, east_offset, 0)
        return grid, offsets, waypoints

    def getRandomGoal(self, grid):
        # fieldGen = Field2DGen(grid)
        # field = fieldGen.buildToClosestBoundary(lambda v: v >= 0.5)
        # grid_goal = (grid_start[0] + 10, grid_start[1] + 10)
        caps = (
            np.size(grid, 0),
            np.size(grid, 1)
        )
        while True:
            proposal = tuple(map(lambda v: random.randint(0, v), caps))
            if grid[proposal[0], proposal[1]] == 0:
                return proposal


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning_RRTStar(conn)
    time.sleep(1)

    drone.start()
