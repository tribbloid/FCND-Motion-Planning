import random

from dataclasses import dataclass
import numpy as np
import networkx as nx
from bresenham import bresenham


@dataclass
class TreeNode(object):
    wp: tuple = (0, 0)

    def __post_init__(self):
        self.vec = np.array(self.wp)

    def __hash__(self):
        return self.wp.__hash__()

    def cost(self, other: 'TreeNode'):
        return np.linalg.norm(self.vec - other.vec)


@dataclass
class RRTStar:
    grid: np.ndarray

    def run(self, start, goal, h, r):
        grid = self.grid
        tree: nx.DiGraph = nx.DiGraph()
        startNode = TreeNode(start)
        goalNode = TreeNode(goal)
        tree.add_node(startNode)

        def sample() -> TreeNode:
            n = random.randint(0, np.size(grid, 0) - 1)
            e = random.randint(0, np.size(grid, 1) - 1)
            return TreeNode((n, e))

        def findNearestSet(p: TreeNode):
            ll = list(filter(lambda v: v.cost(p) <= r, tree.nodes))

            if len(ll) <= 0:
                closest = min(tree.nodes, key=lambda v: v.cost(p))
                ll.append(closest)
            return ll

        def isReachable(a: TreeNode, b: TreeNode) -> bool:
            lines: list = list(bresenham(*a.wp, *b.wp))
            colliding = list(filter(lambda v: grid[v] >= h, lines))
            return len(colliding) == 0

        def rewire(p: TreeNode, nearests : set):
            pass

        finished = False
        while not finished:
            p = sample()
            nearests = findNearestSet(p)

            reachable = list(filter(lambda v: isReachable(v, p), nearests))
            if len(reachable) == 0:
                pass
            else:
                closestReachable = min(reachable, key=lambda v: v.cost(p))

                tree.add_node(p)
                tree.add_edge(closestReachable, p, attr={'cost', closestReachable.cost(p)})

                rewire(p, nearests)

                if isReachable(p, goalNode):
                    tree.add_node(p)
                    tree.add_edge(p, goalNode)
                    finished = True

        path = nx.shortest_path(tree, startNode, goalNode, 'cost')
        pathWP = list(map(lambda v: v.wp, path))

        return tree, pathWP
