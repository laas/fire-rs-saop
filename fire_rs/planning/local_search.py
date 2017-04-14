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

    def __init__(self, plans: 'List(Plan)', parent: 'SearchNode'=None, time_budget=None,
                 cost_function: 'Callable[[SearchNode], float]'=None):
        assert parent is not None or (time_budget is not None and cost_function is not None)
        self.plans = plans
        if parent is not None:  # inherit default properties from parent
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
    def is_valid(self) -> bool:
        """Returns True if all plans respect the time budget."""
        return all([p.duration <= self.time_budget for p in self.plans])

    @property
    def cost(self) -> float:
        """Lazily computes the cost of this node, based on the given cost function passed in __init__()"""
        if self._cost is None:
            self._cost = self.cost_function(self)
        return self._cost

    def is_better_than(self, other: 'SearchNode') -> bool:
        """Returns true if this node is strictly better than the 'other'.
        A node is strictly better if its cost is lower or if it has the same cost but lower duration."""
        if self.cost < other.cost - 0.0001:
            return True
        elif self.cost <= other.cost and self.duration < other.duration - 0.0001:
            return True
        else:
            return False

    @property
    def duration(self) -> float:
        """Sum of the duration of all subplans."""
        return sum([x.duration for x in self.plans])

    @property
    def length(self) -> int:
        """Total number of waypoints in all subplans."""
        return sum([x.length for x in self.plans])

    def all_waypoints(self) -> 'List':
        """Returns the union of waypoints of all subplans."""
        return reduce(lambda x, y: x+y, map(lambda x: x.waypoints, self.plans), [])

    def insert(self, plan_id: int, insertion_id: int, obs) -> 'SearchNode':
        """Convenience function, that creates a new search node by inserting a new observation 
        at the given place of the given plan.
        """
        subplans = list(self.plans)
        subplans[plan_id] = subplans[plan_id].extend(obs, insertion_id)
        return SearchNode(plans=subplans, parent=self)

    def remove(self, plan_id: int, remove_loc: int, number=1) -> 'SearchNode':
        """Remove a given number nodes in the plan_id^th plan, starting from the remove_loc^th waypoint.
        The first and and last waypoint are kept untouched.
        If the penultimate waypoint is removed, removal continues from the second one onward.
        """
        assert 0 < remove_loc < self.plans[plan_id].length-1,\
            "Invalid remove, note that the start and end positions can not be removed with this method"
        subplans = list(self.plans)
        iters = 0
        while iters < number:
            iters += 1
            if subplans[plan_id].length == 2:
                break  # nothing left to remove beside start/end positions
            elif remove_loc < subplans[plan_id].length-1:
                # inside the main plan
                subplans[plan_id] = subplans[plan_id].remove(remove_loc)
            else:
                # we have already removed the last (non-end) waypoint, remove the first non-start position
                subplans[plan_id] = subplans[plan_id].remove(1)
        return SearchNode(subplans, parent=self)


def descent(node: 'SearchNode',
            neighborhood: 'Neighborhood',
            max_iters=np.inf,
            continue_on_first_improvement=False,
            on_improvement: 'Callable[[SearchNode]]'=None) -> 'SearchNode':
    """
    Local search procedure that iteratively selects the most improving node in the neighbors of the last selected node.
    The best found node is returned if it has no better neighbors.
    
    :param node: Node from which to start exploring.
    :param neighborhood: Used to generate all neighbors of a given SearchNode
    :param max_iters: Stop search after a given number of iterations [optional]
    :param continue_on_first_improvement: If true, the first improving neighbor will be selected as the next pivot
     [defaults to false]
    :param on_improvement: A function to invoke at each iteration (when the best node is improved) [optional]
    :return: 
    """
    assert node.is_valid, "Invalid node given to start the descent."
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


def vns(node: 'SearchNode',
        neighborhoods: 'List(Neighborhood)',
        shake_func: 'Callable[[SearchNode], SearchNode]',
        max_iters,
        on_global_improvement: 'Callable[[SearchNode]]'=lambda x: None,
        on_pivot_change: 'Callable[[SearchNode]]'=lambda x: None) -> 'SearchNode':
    """Text-book algorithm for variable neighborhood search."""
    assert node.is_valid, "Invalid node given to start local search procedure"
    best = x = node
    current_iteration = 0

    while current_iteration < max_iters:
        current_iteration += 1
        print("VNS iter #{}".format(current_iteration))
        k = 0  # index of the current neighborhood
        if current_iteration > 1:
            # shake current pivot if it is not the initial node
            x = shake_func(x)
            on_pivot_change(x)
            if x.is_better_than(best):
                best = x
                on_global_improvement(best)

        # try all neighborhoods until none can provide an improved solution
        while k < len(neighborhoods):
            print("  VNS neighborhood: #{}".format(k))
            # do single step descent with the current neighborhood
            x2 = descent(x, neighborhoods[k], max_iters=1, on_improvement=on_pivot_change)
            if x2.is_better_than(x):
                # pivot was improved, select x2 has new pivot and select the first neighborhood
                x = x2
                on_pivot_change(x)
                k = 0
            else:
                # no improvement, go to next neighborhood
                k += 1
            if x.is_better_than(best):
                best = x
                on_global_improvement(best)

    on_pivot_change(best)
    return best


def construct(node: 'SearchNode', candidate_obs, cost_func: 'Callable[[List], float]') -> 'SearchNode':
    """Constructs an initial solution.
    Right now this simply the N most interesting node (where N is the number of UAVs) and allocates one 
    to each UAV.
    """
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

# global counter to avoid plotting at each call to the plot function
cur_plot = 0


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
    axes.set_xlim(min(xs), max(xs))
    axes.set_ylim(min(ys), max(ys))
    for obs in observation_candidates:
        axes.plot(obs[0], obs[1], 'ro')

    # list of patches (i.e. drawing of plans) that are currently displayed
    visible_plan_patches = []

    def draw_solution(node: 'SearchNode', plot_every=1):
        global cur_plot
        cur_plot += 1
        # only plot if cur_plot is a multiple of 'plot_every'
        if (cur_plot % plot_every) != 0:
            return
        # remove previously displayed plans
        for p in visible_plan_patches:
            p.set_visible(False)
        visible_plan_patches.clear()

        # plot all subplans
        for subplan in node.plans:
            _, patch = subplan.plot(axes=axes, blocking=False)
            visible_plan_patches.append(patch)
        plt.show(block=False)
        plt.pause(0.001)

    start_time = time.clock()
    plot_online = True

    # two skywalker UAVs, one based in (0,0) and one in (500,500)
    initial_plans = [Plan(skywalker, [(0, 0), (0, 0)]), Plan(skywalker, [(500, 500), (500, 500)])]

    # maximum duration of each subplan
    time_budget = 100
    init_node = SearchNode(plans=initial_plans, time_budget=time_budget, cost_function=lambda x: cost_func(x.all_waypoints()))
    if plot_online:
        draw_solution(init_node)

    # construct initial solution plan (mostly useful for descent)
    init_node = construct(init_node, observation_candidates, cost_func)

    if plot_online:
        draw_solution(init_node)

    algo = 'vns'
    if algo == 'descent':
        # neighborhood composed of the three local moves
        neighborhood = CombinedNeighborhood([
            Insert(observation_candidates, max_neighbors=50),
            Swap(max_neighbors=50),
            Relocate(max_neighbors=50)])
        if plot_online:
            plan = descent(init_node, neighborhood,
                           on_improvement=lambda x: draw_solution(x, plot_every=10))
        else:
            plan = descent(init_node, neighborhood)

    elif algo == 'vns':  # Variable Neighborhood Search
        # first neighborhood, that optimizes existing plans without adding nodes to it
        n1 = CombinedNeighborhood([Swap(max_neighbors=50), Relocate(max_neighbors=50)])

        # second neighborhood that provides insertion of new nodes in the subplans
        n2 = Insert(observation_candidates, max_neighbors=50)

        # how many time to restart
        max_iters = 15

        # shake function that simply removes up to 5 consecutive waypoints in each subplan
        def shake(n: 'SearchNode') -> 'SearchNode':
            shaked = n
            for i in range(0, len(n.plans)):
                rm_loc = random.choice(range(1, shaked.plans[i].length-1))
                shaked = shaked.remove(i, rm_loc, number=5)
            return shaked

        # run variable neighborhood search and record result in plan
        if plot_online:
            plan = vns(init_node, [n1, n2], shake_func=shake, max_iters=max_iters,
                       on_pivot_change=lambda x: draw_solution(x, plot_every=40))
        else:
            plan = vns(init_node, [n1, n2], shake_func=shake, max_iters=max_iters)
    else:
        print("Error: unknown elgorithm: {}".format(algo))
        exit(1)

    end_time = time.clock()
    print(plan)
    print("Solving time: {}s".format(end_time - start_time))
    draw_solution(plan)

    plt.show(block=False)
    print("done")  # Simply to put a break point
    plt.show(block=True)


if __name__ == '__main__':
    main()
