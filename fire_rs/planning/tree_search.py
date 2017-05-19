import random
import numpy as np
import itertools
import collections
import scipy.spatial
from  scipy.spatial.distance import cdist, euclidean
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

    def __init__(self, points, observation_radius=10, lookahead_distance=20, depth=3,
                 available_commands=np.array([0., -np.pi/6, +np.pi/6]), command_execution_time=2.5, time_step=0.1,
                 default_command=0.):
        self.points = points
        self.observation_radius = observation_radius  # Assume we observe a circular area around the uav
        self.lookahead_distance = lookahead_distance
        self.depth = depth
        self.available_commands = available_commands
        self.command_execution_time = command_execution_time
        self.time_step = time_step
        self.default_command = default_command
        self.observed_mask = np.zeros((self.points.shape[0],), dtype=np.bool)

    def close_to(self, position, distance, observation_mask=None):
        """Get the index of 'points' closer than 'distance' to 'position'"""
        if observation_mask is None:
            observation_mask = np.zeros((self.points.shape[0], 1), dtype=np.bool)
        closeto = np.array([True if observation_mask[i] == True or euclidean(self.points[i], position) <= distance else\
                            False for i in range(observation_mask.shape[0])], dtype=np.bool)
        return closeto

    def far_from(self, position, distance, observation_mask=None):
        """Get the index of 'points' closer than 'distance' to 'position'"""
        if observation_mask is None:
            observation_mask = np.zeros((self.points.shape[0], 1), dtype=np.bool)
        farfrom = np.array([True if observation_mask[i] == True or euclidean(self.points[i], position) >= distance else\
                            False for i in range(observation_mask.shape[0])], dtype=np.bool)
        return farfrom

    def in_front_of(self, position, direction, observation_mask=None):
        if observation_mask is None:
            observation_mask = np.zeros((self.points.shape[0], 1), dtype=np.bool)
        ab = np.array([np.cos(direction), np.sin(direction)])
        infrontof = np.array([True if observation_mask[i] == True or ab @ (position - self.points[i]) < 0 else False\
                             for i in range(observation_mask.shape[0])], dtype=np.bool)
        return infrontof

    def estimate_heuristic(self, uav_state, observation_mask=None):
        close_mask = self.close_to(uav_state[0:2], self.lookahead_distance, observation_mask)
        infront_mask = self.in_front_of(uav_state[0:2], uav_state[2], observation_mask)
        filtered = self.points[np.logical_and(np.logical_xor(observation_mask, close_mask),
                                              np.logical_xor(observation_mask, infront_mask))]
        h = len(filtered)
        return h, filtered

    def best_combination(self, virtual_uav: FixedWing):
        best_combination = (None, 0, self.observed_mask.copy())  # Sequence of commands, cumulated value, observed points
        original_observed_mask = self.observed_mask.copy()

        initial_state = virtual_uav.state.copy()
        initial_input = virtual_uav.input.copy()

        # For every possible combination of movements
        for input_sequence in itertools.product(*itertools.repeat(self.available_commands, self.depth)):
            observed_mask = original_observed_mask.copy()
            virtual_uav.reset_state(initial_state)
            virtual_uav.input = initial_input.copy()
            # Evaluate how many points are seen while applying the inputs
            for inp in input_sequence:
                virtual_uav.input += inp
                # But only if we say to go straight
                if np.isclose(inp, 0.):
                    for n, state in enumerate(virtual_uav.run(self.time_step,
                                                              int(self.command_execution_time / self.time_step))):
                        if n % 2:
                            closeto = self.close_to(state[0:2], self.observation_radius, observed_mask)
                            observed_mask |= closeto
                else:
                    for n, state in enumerate(virtual_uav.run(self.time_step,
                                                              int(self.command_execution_time / self.time_step))):
                        if n % 2 :
                            closeto = self.close_to(state[0:2], self.observation_radius, observed_mask)
                            observed_mask |= closeto
                    # consume(virtual_uav.run(self.time_step, int(self.command_execution_time / self.time_step)))

            # Compute how many points we observed during this input sequence
            visited_value = np.count_nonzero(np.logical_xor(original_observed_mask, observed_mask))
            # Estimate how many points could be reached in the next search
            # (those in front of the uav and closer than a threshold, already visited discarded)
            heuristic_value, points_heuristic = self.estimate_heuristic(
                virtual_uav.state, np.logical_xor(original_observed_mask, observed_mask))

            # If the result is improved, save it
            cumulated_value = visited_value + heuristic_value
            if (cumulated_value) > best_combination[1]:
                best_combination = (input_sequence, cumulated_value, observed_mask)
                logging.debug("Found a better combination: {}\n\t value: {}\n\t n visited: {}".format(
                             str(best_combination[0]), best_combination[1], np.count_nonzero(best_combination[2])))

        # Once all combinations were tested...
        if best_combination[0] is None:
            default_input = np.ones(self.depth) * self.default_command
            best_combination = (default_input, 0., self.observed_mask.copy())
            logging.info("No suitable input combination. Default to: {}".format(best_combination[0]))
        else:
            # Mark the points seen from applying the best combination
            self.observed_mask = best_combination[2].copy()
            logging.info("Best combination: {}\n\t value: {}\n\t n visited: {}".format(
                str(best_combination[0]), best_combination[1], np.count_nonzero(np.logical_xor(original_observed_mask,
                                                                                               best_combination[2]))))

        return best_combination


def main():
    logging.basicConfig(level=logging.INFO)
    camera_fov = (10, 10)  # Horizontal, Vertical
    points_to_observe, labels_true = make_blobs(n_samples=200, cluster_std=3, centers=20,
                                                random_state=20)
    points_to_observe = points_to_observe * 10
    points_values = np.ones(points_to_observe.shape[0])

    axes = plt.gca()
    axes.set_xlim([-150, 150])
    axes.set_ylim([-150, 150])

    plt.scatter(points_to_observe[...,0], points_to_observe[...,1])
    plt.show()

    uav = FixedWing(5., np.pi / 6, initial_state=np.array([np.random.random_sample()*200-100,
                                                           np.random.random_sample()*200-100,
                                                           np.random.random_sample()*2*np.pi, 0.]))
    uav.input = np.array([0.,])# [x, y, ψ, ϕ]

    # draw UAV
    plt.plot(uav.state[0], uav.state[1], 'o', markerfacecolor='red',
             markeredgecolor='red', markersize=3)
    plt.plot([uav.state[0], uav.state[0]+5*np.cos(uav.state[2])],
             [uav.state[1], uav.state[1]+5*np.sin(uav.state[2])],
             '-', c='red')

    searcher = PathTreeSearch(points_to_observe)
    for iterations in range(100):
        # h, points_in_front = searcher.estimate_heuristic(uav.state.copy())
        # for point_in_front in points_in_front:
        #     plt.plot(point_in_front[0], point_in_front[1], 'o', markerfacecolor='orange',
        #              markeredgecolor='orange', markersize=5)
        #     plt.show()

        commands = searcher.best_combination(uav.copy())

        uav_commands = commands[0]
        visited_points = points_to_observe[commands[2]] if len(commands[2]) > 0 else None
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