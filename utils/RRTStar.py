import math
import os
import random
from typing import List, Set

import networkx as nx
import numpy as np
from bresenham import bresenham
from dataclasses import dataclass

COST = 'cost'
MAX_ITR = 2000


@dataclass
class TreeNode(object):
    wp: tuple = (0, 0)

    def __post_init__(self):
        self.vec = np.array(self.wp)
        self.cumCostMnemonic = None

    def __hash__(self):
        return self.wp.__hash__()

    def cost(self, next: 'TreeNode'):
        return math.sqrt(np.linalg.norm(self.vec - next.vec))

    # calculate recursively (lazily)
    def cumCost(self, tree: nx.DiGraph):
        if self.cumCostMnemonic is not None:
            return self.cumCostMnemonic
        parents = list(tree.predecessors(self))
        size = len(parents)
        result = 0
        if size == 0:
            pass
        elif size == 1:
            parent = parents[0]
            parentCumCost = parent.cumCost(tree)
            result = parentCumCost + parent.cost(self)
        else:
            raise os.error("IMPOSSIBLE")
        self.cumCostMnemonic = result
        return result


@dataclass
class RRTStar:
    grid: np.ndarray
    h: int = 0.5

    def sample(self, openSet: Set[tuple]) -> TreeNode:
        v = random.sample(openSet, 1)[0]
        openSet.remove(v)
        return TreeNode(v)

    def isReachable(self, a: TreeNode, b: TreeNode) -> bool:
        lines: list = list(bresenham(*a.wp, *b.wp))
        colliding = list(filter(lambda v: self.grid[v] >= self.h, lines))
        return len(colliding) == 0

    def rewire(self, p: TreeNode, nearests: List[TreeNode], tree: nx.DiGraph):
        for nearest in nearests:
            if self.isReachable(p, nearest):
                oldCumCost = nearest.cumCost(tree)
                newCumCost = p.cumCost(tree) + p.cost(nearest)
                if newCumCost < oldCumCost:
                    old_ps = list(tree.predecessors(nearest))
                    for old_p in old_ps:
                        tree.remove_edge(old_p, nearest)
                    tree.add_edge(p, nearest)
                    nearest.cumCostMnemonic = newCumCost
                    ss = nx.dfs_preorder_nodes(tree, nearest)
                    for successor in ss:
                        successor.cumCostMnemonic = None

    def run(self, start, goal, gamma=1000):
        grid = self.grid
        tree: nx.DiGraph = nx.DiGraph()
        startNode = TreeNode(start)
        goalNode = TreeNode(goal)
        tree.add_node(startNode)

        ii = np.argwhere(grid < self.h)
        openSet = set(list(map(lambda v: tuple(v), ii)))

        n = 1
        def findNearestSet(p: TreeNode):
            nonlocal n
            r = gamma * ((math.log(n) / n) ** (1/2))
            ll = list(filter(lambda v: v.cost(p) <= r, tree.nodes))

            if len(ll) <= 0:
                closest = min(tree.nodes, key=lambda v: v.cost(p))
                ll.append(closest)
            return ll

        decreasingCosts = []
        firstSolution = None
        def stoppingCondition(p: TreeNode):
            nonlocal decreasingCosts
            nonlocal firstSolution

            if self.isReachable(p, goalNode) and firstSolution is None:
                tree.add_node(goalNode)
                tree.add_edge(p, goalNode)
                firstSolution = n

            if firstSolution is not None:
                currentCost = goalNode.cumCost(tree)
                decreasingCosts.append(currentCost)

                print("iteration", n, "\tsolved: cost =", currentCost)
                # stop when the cost doesn't decrease for 10 iterations
                if len(decreasingCosts) > 100 and decreasingCosts[-100] == currentCost:
                    return True
                elif len(decreasingCosts) >= 1000:
                    return True
                else:
                    return False
            else:
                print("iteration", n)
                return False

        isSolved = False
        while not isSolved and n < MAX_ITR:
            p = self.sample(openSet)
            nearests = findNearestSet(p)

            reachable = list(filter(lambda v: self.isReachable(v, p), nearests))
            if len(reachable) == 0:
                pass
            else:
                closestParent = min(reachable, key=lambda v: v.cost(p))

                cost = closestParent.cost(p)
                tree.add_node(p)
                tree.add_edge(closestParent, p, attr={'cost', cost})

                self.rewire(p, nearests, tree)

                isSolved = stoppingCondition(p)

            n += 1

        if isSolved:
            path = nx.shortest_path(tree, startNode, goalNode, COST)
            pathWP = list(map(lambda v: v.wp, path))

            print("WayPoints: ", pathWP)

            return tree, pathWP
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
            return tree, None
