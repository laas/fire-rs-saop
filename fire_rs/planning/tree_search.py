import random
import numpy as np
import itertools
import collections
import scipy.spatial
from  scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.datasets.samples_generator import make_blobs
import logging
from fire_rs.planning.uav import FixedWing

# Faster way to advance iterators
def consume(iterator, n=None):
    "Advance the iterator n-steps ahead. If n is none, consume entirely."
    # Use functions that consume iterators at C speed.
    if n is None:
        # feed the entire iterator into a zero-length deque
        collections.deque(iterator, maxlen=0)
    else:
        # advance to the empty slice starting at position n
        next(itertools.islice(iterator, n, n), None)



class PathTreeSearch:

    def __init__(self, points, observation_radius=10 ,depth=4, available_commands=np.array([-np.pi/6, 0., +np.pi/6]),
                 command_execution_time=2., time_step=0.1, default_command=+np.pi/6.):
        self.points = points
        self.observation_radius = observation_radius  # Assume we observe a circular area around the uav
        self.depth = depth
        self.available_commands = available_commands
        self.command_execution_time = command_execution_time
        self.time_step = time_step
        self.default_command = default_command
        self.observed_mask = np.zeros(self.points.shape[0], dtype=np.bool)

    def best_combination(self, uav: FixedWing):
        a_uav = uav.copy()
        best_combination = (None, 0, self.observed_mask.copy())  # Sequence of commands, cumulated value, observed points

        initial_state = uav.state.copy()
        initial_input = uav.input.copy()

        for input_combination in itertools.product(*itertools.repeat(self.available_commands, self.depth)):
            cumulated_value = 0
            observed_mask = self.observed_mask.copy()
            a_uav.reset_state(initial_state)
            a_uav.input = initial_input.copy()
            for inp in input_combination:
                a_uav.input += inp
                if np.isclose(inp, 0.):
                    for n, state in a_uav.run(self.time_step, int(self.command_execution_time / self.time_step)):
                        observed_mask |= np.array(list(map(lambda d: d <= self.observation_radius,
                                                           cdist(self.points, [state[0:2]]).flat)))
                else:
                    # for n, state in a_uav.run(self.time_step, int(self.command_execution_time / self.time_step)):
                    #     observed_mask = observed_mask | np.array(
                    #         list(map(lambda d: d <= self.observation_radius, cdist(self.points, [state[0:2]]).flat)))
                    consume(a_uav.run(self.time_step, int(self.command_execution_time / self.time_step)))

            cumulated_value = np.count_nonzero(observed_mask)

            if cumulated_value > best_combination[1]:
                best_combination = (input_combination, cumulated_value, observed_mask)
                logging.info("Found a better combination: {}, {}, {}".format(
                             str(input_combination), str(cumulated_value), str(self.points[observed_mask])))

        if best_combination[0] is None:
            default_input = np.ones(self.depth) * self.default_command
            default_input[0] = 0.
            best_combination = (default_input, 0., None)
            logging.info("No suitable input combination. Default to: {}, {}, {}".format(
                        str(input_combination), str(cumulated_value), str(self.points[observed_mask])))

        return best_combination


def main():
    logging.basicConfig(level=logging.DEBUG)
    camera_fov = (10, 10)  # Horizontal, Vertical
    points_to_observe, labels_true = make_blobs(n_samples=200, cluster_std=3, centers=20,
                                                random_state=20)
    points_to_observe = points_to_observe * 10
    points_values = np.ones(points_to_observe.shape[0])

    plt.scatter(points_to_observe[...,0], points_to_observe[...,1])
    plt.show()

    uav = FixedWing(5., np.pi / 6, initial_state=np.array([np.random.random_sample()*200-100,
                                                           np.random.random_sample()*200-100, np.random.random_sample()*2*np.pi, 0.]))
    uav.input = np.array([0.,])# [x, y, ψ, ϕ]

    searcher = PathTreeSearch(points_to_observe)
    for iterations in range(100):
        commands = searcher.best_combination(uav.copy())

        uav_commands = commands[0]
        visited_points = points_to_observe[commands[2]]
        for i in range(searcher.depth):
            uav.input += uav_commands[i]
            local_traj = []
            for __ in np.arange(0., searcher.command_execution_time, searcher.time_step):
                uav.step(searcher.time_step)
                local_traj.append(uav.output[0:2])
            local_traj = np.array(local_traj)
            plt.plot(local_traj[..., 0], local_traj[..., 1])
        if visited_points is not None:
            plt.plot(visited_points[:, 0], visited_points[:, 1], 'o', markerfacecolor='green',
                     markeredgecolor='green', markersize=5)
            plt.show()






if __name__ == '__main__':


    main()

    print("")