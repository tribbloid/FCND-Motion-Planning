import random

from dataclasses import dataclass
import numpy as np
from bresenham import bresenham


@dataclass
class TreeNode(object):
    wp: tuple = (0, 0)
    children: list['TreeNode'] = list()

    def __post_init__(self):
        self.vec = np.array(self.wp)

    def cost(self, other: 'TreeNode'):
        return np.linalg.norm(self.vec - other.vec)

def rrtStar(grid, start, goal, h, r):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    tree: set = set(TreeNode(start))

    def sample() -> TreeNode:
        n = random.randint(0, np.size(grid, 0))
        e = random.randint(0, np.size(grid, 1))
        return TreeNode(n, e)

    finished = False
    while(finished):
        p = sample()
        closestSet: set = filter(lambda v: v.cost(p) <= r, tree)

        if closestSet.__len__() <= 0:
            closest = min(tree, key=lambda v: v.cost(p))
            closestSet.add(closest)

        def isReachable(parent: TreeNode) -> bool:
            lines = list(bresenham(*parent.wp, *p.wp))
            colliding = list(filter(lambda v: grid(lines) >= h, lines))

            return len(colliding) == 0

        reachable = list(filter(isReachable, closestSet))
        if len(reachable) == 0:
            pass;
        else:
            clo

        closestReachable = min(reachable, key=lambda v: v.cost(p))
        closestReachable.children += p



    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost