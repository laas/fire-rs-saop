import random
from functools import reduce

from typing import List, Callable

import numpy as np
from scipy.spatial.distance import cdist

from fire_rs.planning.neighborhoods import *
from fire_rs.planning.plan import Plan
from fire_rs.planning.uav import skywalker


class SearchNode:
    """A search node is composed of a sequence of Plans (one for each UAV) and provides helper functions
    to access some of their properties."""

    def __init__(self, plans: 'List(Plan)', parent=None, time_budget=None, cost_function: 'Callable[[SearchNode], float]'=None):
        assert parent is not None or time_budget is not None
        self.plans = plans
        if parent is not None:
            self.time_budget = parent.time_budget
            self.cost_function = parent.cost_function
        if time_budget is not None:
            self.time_budget = time_budget
        if cost_function is not None:
            self.cost_function = cost_function
        self._cost = None

    def __repr__(self):
        ret = ""
        for subplan in self.plans:
            ret += str(subplan) + "\n"
        return ret

    @property
    def is_valid(self) -> 'bool':
        """Returns True if all plans respect the time budget."""
        return all([p.duration <= self.time_budget for p in self.plans])

    @property
    def cost(self) -> 'float':
        """Lazily computes the cost of this node, based on the given cost function passed in __init__()"""
        if self._cost is None:
            self._cost = self.cost_function(self)
        return self._cost

    def is_better_than(self, other: 'SearchNode') -> 'bool':
        """Returns true if this node is strictly better than the 'other'.
        A node is strictly better if its cost is lower or if it has the same cost but lower duration."""
        if self.cost < other.cost:
            return True
        elif self.cost <= other.cost and self.duration < other.duration:
            return True
        else:
            return False

    @property
    def duration(self):
        """Sum of the duration of all subplans."""
        return np.sum([x.duration for x in self.plans])

    @property
    def length(self):
        """Total number of waypoints in all subplans."""
        return np.sum([x.length for x in self.plans])

    def all_waypoints(self):
        """Returns the union of waypoints of all subplans."""
        return reduce(lambda x, y: x+y, map(lambda x: x.waypoints, self.plans), [])

    def insert(self, plan_id: int, insertion_id: int, obs) -> 'SearchNode':
        """Convenience function, that creates a new search node by inserting a new observation 
        at the given place of the given plan.
        """
        subplans = list(self.plans)
        subplans[plan_id] = subplans[plan_id].extend(obs, insertion_id)
        return SearchNode(plans=subplans, parent=self)


def descent(node: 'SearchNode',
            neighborhood: 'Neighborhood',
            max_iters=np.inf,
            continue_on_first_improvement=False,
            on_improvement: 'Callable[[SearchNode]]'=None) -> 'SearchNode':
    """
    Local search procedure.
    :param node: Node from which to start exploring.
    :param neighborhood: Used to generate all neighbors of a given SearchNode
    :param max_iters: Stop search after a given number of iterations [optional]
    :param continue_on_first_improvement: If true, the first improving neighbor will be selected as the next pivot
     [defaults to false]
    :param on_improvement: A function to invoke at each iteration (when the best node is improved) [optional]
    :return: 
    """
    assert node.is_valid
    best = node
    improved = True
    num_iters = 0
    while improved and num_iters < max_iters:
        num_iters += 1
        improved = False
        neighbors = neighborhood.neighbors(best)
        random.shuffle(neighbors)
        for neighbor in neighbors:
            if neighbor.is_better_than(best):
                best = neighbor
                improved = True
                if continue_on_first_improvement:
                    break  # stop on first improvement
        if improved:
            print("Improved plan cost: {}".format(best.cost))
            if on_improvement is not None:
                on_improvement(best)
    return best


def construct(node: 'SearchNode', candidate_obs, cost_func: 'Callable[[List], float]') -> 'SearchNode':
    while not all([subplan.length > 2 for subplan in node.plans]):
        best_obs = None
        best_obs_cost = np.inf
        for obs in random.sample(candidate_obs, 30):
            if cost_func(node.all_waypoints() + [obs]) < best_obs_cost:
                best_obs = obs
                best_obs_cost = cost_func(node.all_waypoints() + [obs])
        best_node = None
        for i_subplan in range(0, len(node.plans)):
            if node.plans[i_subplan].length > 2:
                continue
            tmp = node.insert(i_subplan, 1, best_obs)
            if not tmp.is_valid:
                continue
            if best_node is None or tmp.duration < best_node.duration:
                best_node = tmp
        node = best_node if best_node is not None else node
    return node


def main():
    import matplotlib.pyplot as plt
    import time
    num_candidates = 300
    observation_candidates = list(np.random.randint(0, 500, (num_candidates, 2)))

    def cost_func(observations) -> 'float':
        dists_past = cdist(observation_candidates, observations)
        return np.sum(np.amin(dists_past, 1))

    axes = plt.figure().add_subplot(111)
    xs = [x for (x, y) in observation_candidates]
    ys = [y for (x, y) in observation_candidates]

    def draw_solution(node: 'SearchNode'):
        axes.clear()
        for subplan in node.plans:
            subplan.plot(axes=axes, blocking=False)
        for obs in observation_candidates:
            axes.plot(obs[0], obs[1], 'ro')
        axes.set_xlim(min(xs), max(xs))
        axes.set_ylim(min(ys), max(ys))
        plt.show(block=False)
        plt.pause(0.001)

    start_time = time.clock()
    plot_online = True

    # two skywalker UAVs, one based in (0,0) and one in (500,500)
    initial_plans = [Plan(skywalker, [(0, 0), (0, 0)]), Plan(skywalker, [(500, 500), (500, 500)])]
    time_budget = 100
    init_node = SearchNode(plans=initial_plans, time_budget=time_budget, cost_function=lambda x: cost_func(x.all_waypoints()))
    if plot_online:
        draw_solution(init_node)
    init_node = construct(init_node, observation_candidates, cost_func)
    if plot_online:
        draw_solution(init_node)

    neighborhood = CombinedNeighborhood([
        Insert(observation_candidates, max_neighbors=50),
        Swap(max_neighbors=50),
        Relocate(max_neighbors=50)
    ])

    # plan = local_search(SearchNode([Plan(skywalker, [(0, 0), (0, 0)])]))
    if plot_online:
        plan = descent(init_node, neighborhood, on_improvement=draw_solution)
    else:
        plan = descent(init_node, neighborhood)

    end_time = time.clock()
    print(plan)
    print("Solving time: {}s".format(end_time - start_time))
    draw_solution(plan)

    plt.show(block=False)
    print("done")
    plt.show(block=True)


if __name__ == '__main__':
    main()
