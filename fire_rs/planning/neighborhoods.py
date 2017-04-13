import random
from typing import List

import numpy as np

from fire_rs.planning.local_search import SearchNode
from fire_rs.planning.plan import Plan


def plan_extension(plan: 'Plan', waypoint):
    """Helper function that tries all extensions of a plan with a given observation 
    and returns the extension with the least duration
    """
    best_plan = None
    for i in range(1, plan.length):  # exclude start and end positions
        p = plan.extend(waypoint, i)
        if best_plan is None or p.duration < best_plan.duration:
            best_plan = p
    return best_plan


class Neighborhood:
    """Abstract class defining the interface of a neighborhood.
    A neighborhood essentially provides a method to generate neighbors of a given search node by applying 
    local changes."""

    def __init__(self):
        self._computed_neighborhoods = 0
        self._evaluated_neighbors = 0

    def neighbors(self, node: 'SearchNode') -> 'List(SearchNode)':
        return []

    @property
    def computed_neighborhoods(self):
        return self._computed_neighborhoods

    @property
    def evaluated_neighbors(self):
        return self._computed_neighborhoods


class CombinedNeighborhood(Neighborhood):
    """Helper class providing a neighborhood composed of the union of other neighborhoods."""

    def __init__(self, neighborhoods):
        super(CombinedNeighborhood, self).__init__()
        self.neighborhoods = neighborhoods

    def neighbors(self, node: 'SearchNode') -> 'List(SearchNode)':
        """Simply combine all sub-neighborhoods."""
        return [neighbor for neighborhood in self.neighborhoods for neighbor in neighborhood.neighbors(node)]

    @property
    def computed_neighborhoods(self):
        return sum([x.computed_neighborhoods for x in self.neighborhoods])

    @property
    def evaluated_neighbors(self):
        return sum([x.evaluated_neighbors for x in self.neighborhoods])


class Insert(Neighborhood):
    """Local search move that inserts a new observation (from a given list) into a trajectory."""

    def __init__(self, observation_candidates, max_neighbors=np.inf):
        super(Insert, self).__init__()
        self.observation_candidates = observation_candidates
        self.max_neighbors = max_neighbors

    def neighbors(self, node: 'SearchNode') -> 'List(SearchNode)':
        self._computed_neighborhoods += 1
        neighborhood_size = len(self.observation_candidates) * node.length
        neighbors = []

        if neighborhood_size < self.max_neighbors:
            # we can generate all neighbors exhaustively
            candidates = [(plan_index, obs)
                          for plan_index in range(0, len(node.plans))
                          for obs in self.observation_candidates]
        else:
            # limited number of neighbors, just pick 'max_neighbors' randomly
            candidates = [(random.choice(range(0, len(node.plans))), random.choice(self.observation_candidates))
                          for i in range(0, self.max_neighbors)]

        for (plan_index, obs) in candidates:
            # neighbor built by inserting 'obs' in 'self.plan[plan_index]' at the place where plan duration is minimized
            self._evaluated_neighbors += 1
            subplans = list(node.plans)  # copy of the node's plans
            # try insertion everywhere and select the best resulting plan
            best_subplan = plan_extension(subplans[plan_index], obs)
            if best_subplan is not None:
                # replace previous subplan with the new one
                subplans[plan_index] = best_subplan
                neighbor = SearchNode(subplans, parent=node)

                # if the node is valid, add it to the computed neighborhood
                if neighbor.is_valid:
                    neighbors.append(neighbor)
        return neighbors


class Swap(Neighborhood):
    """Local search move that tries to swap two waypoints in any given trajectories."""

    def __init__(self, max_neighbors=np.inf):
        super(Swap, self).__init__()
        self.max_neighbors = max_neighbors

    def neighbors(self, node: 'SearchNode') -> 'List(SearchNode)':
        self._computed_neighborhoods += 1
        neighborhood_size = node.length * (node.length-1)
        if neighborhood_size < self.max_neighbors:
            candidates = [(i, j, i_obs_id, j_obs_id)
                          for i in range(0, len(node.plans))
                          for j in range(i+1, len(node.plans))
                          for i_obs_id in range(1, node.plans[i].length-1)
                          for j_obs_id in range(1, node.plans[j].length-1)]
        else:
            def random_candidate():
                i = random.choice(range(0, len(node.plans)-1))
                j = random.choice(range(i+1, len(node.plans)))
                i_obs_id = random.choice(range(1, node.plans[i].length-1))
                j_obs_id = random.choice(range(1, node.plans[j].length-1))
                return i, j, i_obs_id, j_obs_id
            candidates = [random_candidate() for i in range(0, self.max_neighbors)]

        neighbors = []
        for (i, j, i_obs_id, j_obs_id) in candidates:
            self._evaluated_neighbors += 1
            obs_i = node.plans[i][i_obs_id]
            obs_j = node.plans[j][j_obs_id]
            subplans = list(node.plans)
            # swap obs_i and obs_j
            subplans[i] = subplans[i].remove(i_obs_id)
            subplans[i] = subplans[i].extend(obs_j, i_obs_id)
            subplans[j] = subplans[j].remove(j_obs_id)
            subplans[j] = subplans[j].extend(obs_i, j_obs_id)
            neighbor = SearchNode(subplans, parent=node)

            # only add to select neighbors if the node is valid
            if neighbor.is_valid:
                neighbors.append(neighbor)
        return neighbors


class Relocate(Neighborhood):
    """Local search move that picks an observation in a given path and tries to relocate it elsewhere 
    (in the same plan or another).
    """

    def __init__(self, max_neighbors=np.inf):
        super(Relocate, self).__init__()
        self.max_neighbors = max_neighbors

    def neighbors(self, node: 'SearchNode') -> 'List(SearchNode)':
        self._computed_neighborhoods += 1
        neighborhood_size = node.length * len(node.plans)
        if neighborhood_size < self.max_neighbors:
            # exhaustively generate all candidates
            candidates = [(i, j, i_obs_id)
                          for i in range(0, len(node.plans))
                          for j in range(0, len(node.plans))
                          for i_obs_id in range(1, node.plans[i].length-1)]
        else:
            # randomly pick 'max_neighbors' candidates
            def random_candidate():
                i = random.choice(range(0, len(node.plans)-1))
                j = random.choice(range(0, len(node.plans)))
                i_obs_id = random.choice(range(1, node.plans[i].length-1))
                return i, j, i_obs_id
            candidates = [random_candidate() for i in range(0, self.max_neighbors)]

        neighbors = []
        for (i, j, i_obs_id) in candidates:
            self._evaluated_neighbors += 1
            obs = node.plans[i][i_obs_id]
            subplans = list(node.plans)
            # swap obs for subplans[i] and add it to subplans[j]
            subplans[i] = subplans[i].remove(i_obs_id)
            subplans[j] = plan_extension(subplans[j], obs)
            neighbor = SearchNode(subplans, parent=node)

            # only add to selected neighbors if the node is valid and reduces duration
            if neighbor.is_valid and neighbor.duration < node.duration:
                neighbors.append(neighbor)
        return neighbors

