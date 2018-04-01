import math
import os
import random
from typing import List

import networkx as nx
import numpy as np
from bresenham import bresenham
from dataclasses import dataclass

from utils import grid2Open

COST = 'cost'
MAX_ITR = 2000


@dataclass
class TreeNode(object):
    wp: tuple = None

    def __post_init__(self):
        if self. wp is None:
            self.wp = (0, 0)
        self.vec = np.array(self.wp)
        self.cumCostMnemonic = None

    def __hash__(self):
        return self.wp.__hash__()

    def cost(self, to: 'TreeNode'):
        return math.sqrt(np.linalg.norm(self.vec - to.vec))

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
    gamma = 1000

    @staticmethod
    def condition(v) -> bool:
        return v <= 0.5

    def __post_init__(self):
        self.openSet = set(grid2Open(self.grid, self.condition))

        self.tree: nx.DiGraph = nx.DiGraph()

    def sample(self, goal: TreeNode) -> TreeNode:
        v = random.sample(self.openSet, 1)[0]
        self.openSet.remove(v)
        return TreeNode(v)

    def isReachable(self, a: TreeNode, b: TreeNode) -> bool:
        line = bresenham(*a.wp, *b.wp)
        for pixel in line:
            if not self.condition(self.grid[pixel]):
                return False
        return True

    def selectBestParent(self, p, reachable):
        closestParent = min(reachable, key=lambda v: v.cost(p))
        # closestParent = min(reachable, key=lambda v: v.cumCost(self.tree) + v.cost(p)) # TODO: which one is correct?
        return closestParent

    def rewire(self, p: TreeNode, nearests: List[TreeNode]):
        tree = self.tree
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

    def run(self, start, goal):
        startNode = TreeNode(start)
        goalNode = TreeNode(goal)

        tree = self.tree
        tree.add_node(startNode)

        n = 1

        def findNearestSet(p: TreeNode):
            nonlocal n
            r = self.gamma * ((math.log(n) / n) ** (1/2))
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

            stationaryObjectiveCap = 100

            if self.isReachable(p, goalNode) and firstSolution is None:
                tree.add_node(goalNode)
                tree.add_edge(p, goalNode)
                firstSolution = n

            if firstSolution is not None:
                currentCost = goalNode.cumCost(tree)
                decreasingCosts.append(currentCost)

                print("iteration", n, "\tsolved: cost =", currentCost)
                # stop when the cost doesn't decrease for 10 iterations
                if len(decreasingCosts) > stationaryObjectiveCap and\
                        decreasingCosts[-stationaryObjectiveCap] == currentCost:
                    return True
                    # return False # for debugging only
                elif n >= MAX_ITR - 1:
                    return True
                else:
                    return False
            else:
                print("iteration", n)
                return False

        isSolved = False
        while not isSolved and n < MAX_ITR:
            p = self.sample(goalNode)
            nearests = findNearestSet(p)

            reachable = list(filter(lambda v: self.isReachable(v, p), nearests))
            if len(reachable) == 0:
                print("fruitless iteration", n)
            else:
                closestParent = self.selectBestParent(p, reachable)

                cost = closestParent.cost(p)
                tree.add_node(p)
                tree.add_edge(closestParent, p, attr={'cost', cost})

                self.rewire(p, nearests)

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


class P_RRTStar(RRTStar):
    _lambda: float = 0.05
    k = 10

    # def __post_init__(self):
    #     super(P_RRTStar, self).__post_init__()
    #     self.potentialField =

    def sample(self, goal: TreeNode) -> TreeNode:
        while True:
            vInit = random.sample(self.openSet, 1)[0]
            attraction = (goal.vec - np.array(vInit)) * self._lambda

            v = vInit
            for i in range(0, self.k):
                vDelta = v + attraction
                vNext = tuple(vDelta.astype(np.int))
                # assert (0 <= vNext[0] <= self.grid.shape[0]) and (1 <= vNext[1] <= self.grid.shape[1]),\
                #     "%r = %r + %r, goal=%r" % (vDelta, v, attraction, goal.vec)
                if self.condition(self.grid[vNext]):
                    v = vNext
                else:
                    break

            # print(i)
            if v in self.openSet:
                self.openSet.remove(v)
                return TreeNode(v)
