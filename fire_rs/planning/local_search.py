from typing import List

import numpy as np
from scipy.spatial.distance import cdist

from fire_rs.planning.plan import Plan
from fire_rs.planning.uav import skywalker

num_candidates = 200

past_observations = np.array([(0,0),(5, 5)])
observation_candidates = np.random.randint(0, 500, (num_candidates, 2)) #np.array([(0, 0), (0, 1), (5, 5), (10, 10)])


def min_dist_vector(observations):
    dists_past = cdist(observation_candidates, observations)
    return np.amin(dists_past, 1)


def cost(plan: 'Plan') -> float:
    min_dists_past = min_dist_vector(past_observations)
    min_dists_plan = min_dist_vector(plan.waypoints)
    min_dists = np.minimum(min_dists_past, min_dists_plan)
    return np.sum(min_dists)


def plan_extension(plan, waypoint):
    best_plan = None
    for i in range(1, plan.length):  # exclude start and end positions
        p = plan.extend(waypoint, i)
        if best_plan is None or p.duration < best_plan.duration:
            best_plan = p
    return best_plan


def neighborhood(plan: 'Plan') -> 'List(Plan)':
    neighbors = []
    for obs in observation_candidates:
        neighbors.append(plan_extension(plan, obs))
    return neighbors

time_budget = 150


def local_search():
    best_plan = Plan(skywalker, [(0, 0), (0, 0)])
    best_plan.cost = cost(best_plan)
    print("Initial plan cost: {}".format(best_plan.cost))

    improved = True
    while improved:
        improved = False
        for n in neighborhood(best_plan):
            n.cost = cost(n)
            if n.duration < time_budget and n.cost < best_plan.cost:
                best_plan = n
                print("Improved plan cost: {}".format(best_plan.cost))
                improved = True

    return best_plan








if __name__ == '__main__':
    import matplotlib.pyplot as plt
    plan = local_search()
    print(plan)
    ax = plan.plot(blocking=False)
    for obs in observation_candidates:
        ax.plot(obs[0], obs[1], 'ro')
    plt.show(block=True)
