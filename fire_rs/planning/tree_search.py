import random
import numpy as np
import itertools
import collections
from  scipy.spatial.distance import cdist, euclidean
import matplotlib.pyplot as plt
from sklearn.datasets.samples_generator import make_blobs
import logging
from fire_rs.planning.uav import FixedWing
from fire_rs.display import plot_uav

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

    def __init__(self, points, observation_radius=50, lookahead_distance=100, depth=3,
                 available_commands=np.array([0., -np.pi/6, +np.pi/6]), command_execution_time=1, time_step=0.1,
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

    def close_to(self, position, distance, mask=None):
        """Get a mask of 'points' closer than 'distance' to 'position'
        if mask is set, only True points will be considered for the 'close to' condition"""
        closeto = (cdist(self.points, [position]) <= distance).flatten()
        if mask is not None:
            closeto &= mask
        return closeto

    def far_from(self, position, distance, mask=None):
        """Get a mask of 'points' closer than 'distance' to 'position'
        if mask is set, only True points will be considered for the 'far from' condition"""
        farfrom = (cdist(self.points, [position]) >= distance).flatten()
        if mask is not None:
            farfrom &= mask
        return farfrom

    def in_front_of(self, position, direction, mask=None):
        """Get a mask of 'points' in front of 'position'
        if mask is set, only True points will be considered for the 'in front of' condition"""
        ab = np.array([np.cos(direction), np.sin(direction)])
        # ab @ (position - self.points[i]) is the "inner product"
        infrontof = np.matmul(position - self.points, ab) < 0
        if mask is not None:
            infrontof &= mask
        return infrontof

    def estimate_heuristic(self, uav_state, already_observed=None):
        close_mask = self.close_to(uav_state[0:2], self.lookahead_distance)
        infront_mask = self.in_front_of(uav_state[0:2], uav_state[2])
        filtered = self.points[~already_observed & close_mask & infront_mask]
        h = len(filtered)
        return h, filtered

    def best_combination(self, virtual_uav: FixedWing):
        best_combination_found = False
        lookahead_mod = self.lookahead_distance
        self.lookahead_distance = lookahead_mod

        while not best_combination_found:
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
                                closeto = self.close_to(state[0:2], self.observation_radius, None)
                                observed_mask |= closeto
                    else:
                        # for n, state in enumerate(virtual_uav.run(self.time_step,
                        #                                           int(self.command_execution_time / self.time_step))):
                        #     if n % 2 :
                        #         closeto = self.close_to(state[0:2], self.observation_radius, observed_mask)
                        #         observed_mask |= closeto
                        consume(virtual_uav.run(self.time_step, int(self.command_execution_time / self.time_step)))
                if input_sequence[0] == 0. and input_sequence[1] == 0.:
                    print()
                # Compute how many points we observed during this input sequence
                visited_value = np.count_nonzero(~original_observed_mask&observed_mask)
                # Estimate how many points could be reached in the next search
                # (those in front of the uav and closer than a threshold, already visited discarded)
                heuristic_value, points_heuristic = self.estimate_heuristic(
                    virtual_uav.state, observed_mask)
                print("{}".format(input_sequence, visited_value, heuristic_value))


                # If the result is improved, save it
                cumulated_value = visited_value + heuristic_value
                if (cumulated_value) > best_combination[1]:
                    best_combination = (input_sequence, cumulated_value, observed_mask)
                    logging.debug("Found a better combination: {}\n\t value: {}\n\t n visited: {}".format(
                                 str(best_combination[0]), best_combination[1], np.count_nonzero(best_combination[2])))

            # Once all combinations were tested...
            if best_combination[0] is None:
                best_combination_found = False
                self.lookahead_distance *= 2
                logging.info("No suitable input combination. Increasing lookahead to {}".format(
                    self.lookahead_distance))
            else:
                # Mark the points seen from applying the best combination
                best_combination_found = True
                self.lookahead_distance = lookahead_mod  # Restore original lookahead distance
                self.observed_mask = best_combination[2].copy()
                logging.info("Best combination: {}\n\tvalue: {}\n\tn visited: {}\n\ttotal:{}".format(
                    str(best_combination[0]), best_combination[1],
                    np.count_nonzero(np.logical_xor(original_observed_mask, best_combination[2])),
                    np.count_nonzero(best_combination[2])))

        return best_combination


def main():

    logging.basicConfig(level=logging.INFO)
    camera_fov = (10, 10)  # Horizontal, Vertical
    points_to_observe, labels_true = make_blobs(n_samples=200, cluster_std=3, centers=20,
                                                random_state=1)
    points_to_observe = points_to_observe * 10
    points_values = np.ones(points_to_observe.shape[0])

    axes = plt.gca()
    axes.set_xlim([-150, 150])
    axes.set_ylim([-150, 150])

    plt.scatter(points_to_observe[...,0], points_to_observe[...,1])
    plt.show()

    uav = FixedWing(25., np.pi / 6, initial_state=np.array([np.random.random_sample()*200-100,
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
        # plot_uav(plt.gca(), uav.state, size=1)
        # h, points_in_front = searcher.estimate_heuristic(uav.state.copy())
        # for point_in_front in points_in_front:
        #     plt.plot(point_in_front[0], point_in_front[1], 'o', markerfacecolor='orange',
        #              markeredgecolor='orange', markersize=5)
        #     plt.show()
        orig_lookahead = searcher.lookahead_distance
        commands = searcher.best_combination(uav.copy())
        found = True if not np.isclose(commands[1], 0.) else False
        while not found:
            searcher.lookahead_distance*=1.5
            logging.warning("No combination found. Increasing lookahead distance to {:.0f}".format(
                searcher.lookahead_distance))
            commands = searcher.best_combination(uav.copy())
            found = True if not np.isclose(commands[1], 0.) else False
        searcher.lookahead_distance = orig_lookahead
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


def demo_firefornt_observation():
    import logging
    import numpy as np
    import matplotlib.pyplot as plt
    from fire_rs.geodata import environment
    import fire_rs.firemodel.propagation as propagation
    import fire_rs.display
    import fire_rs.planning.observation_path_search
    import fire_rs.geodata.geo_data
    import os

    def burn(area_bounds, ignition_point, wind):
        """Burn some area from an ignition point with given wind conditions"""
        env = propagation.Environment(area_bounds, wind_speed=wind[0], wind_dir=wind[1])
        f_propagation = propagation.propagate(env, *ignition_point)
        return f_propagation.ignitions()

    def observe(points, uav, color_observed='green'):
        logging.basicConfig(level=logging.INFO)
        points_to_observe = points

        axes = plt.gca()

        searcher = PathTreeSearch(points_to_observe)
        for iterations in range(200):
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
                plt.plot(local_traj[..., 0], local_traj[..., 1], 'black', linewidth=2)
            if visited_points is not None:
                plt.plot(visited_points[:, 0], visited_points[:, 1], 'o', markerfacecolor=color_observed,
                         markeredgecolor=color_observed, markersize=3)
                plot_uav(plt.gca(), uav.state, size=5)
                plt.show()

    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    ignition_point = (100, 100)
    area_wind = (10, np.pi)
    ignition_times = None
    if not os.path.exists('/tmp/ignition.tmp'):
        ignition_times = burn(area, ignition_point, area_wind)
        ignition_times.write_to_file('/tmp/ignition.tmp')
    else:
        ignition_times = fire_rs.geodata.geo_data.GeoData.load_from_file('/tmp/ignition.tmp')

    world = environment.World()
    some_area = world.get_elevation(area)

    uav = FixedWing(25., np.pi / 6, initial_state=np.array([532000.0, 6231800.0, np.random.random_sample() * 2 * np.pi, 0.]))
    uav.input = np.array([0., ])  # [x, y, ψ, ϕ]

    # draw UAV
    plot_uav(plt.gca(), uav.state, size=5)

    n = 0
    colors_burning = ['red', 'firebrick']
    colors_observed = ['darkgreen', 'lightgreen']
    for low, high in zip(range(60, 120, 5), range(65, 125, 5)):
        if low % 10 != 0:
            continue
        ignition_array = ignition_times.data['ignition'] / 60

        selected_points = np.array(list(zip(*np.where((low < ignition_array) & (ignition_array < high)))), dtype=np.float)
        selected_points *= np.array([ignition_times.cell_width, ignition_times.cell_height], dtype=np.float)
        selected_points += np.array([ignition_times.x_offset, ignition_times.y_offset], dtype=np.float)

        try:
            plt.scatter(selected_points[..., 0], selected_points[..., 1], c=colors_burning[n % 2])
            plt.show()
            observe(selected_points, uav, colors_observed[n % 2])
            n += 1
        except ValueError:
            logging.exception("")


if __name__ == '__main__':
    demo_firefornt_observation()
    plt.show()
