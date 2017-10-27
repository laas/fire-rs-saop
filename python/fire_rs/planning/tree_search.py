# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import random
import numpy as np
import itertools
import collections
from  scipy.spatial.distance import cdist, euclidean
import matplotlib.pyplot as plt
from sklearn.datasets.samples_generator import make_blobs
import logging
from fire_rs.planning.uav import FixedWing, wrap_angle_rad
from fire_rs.geodata.display import plot_uav

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

    def estimate_heuristic(self, uav_state, fov=np.pi/6., already_observed=None):
        close_mask = self.close_to(uav_state[0:2], self.lookahead_distance)
        infront_mask = self.in_front_of(uav_state[0:2], uav_state[2]+fov) & \
                       self.in_front_of(uav_state[0:2], uav_state[2]-fov)  # define a field of view of π/3
        filtered = self.points[~already_observed & close_mask & infront_mask]
        h = len(filtered)
        return h, filtered, self.points[~already_observed & infront_mask], self.points[~already_observed & close_mask]

    def best_combination(self, virtual_uav: FixedWing):
        best_combination_found = False
        lookahead_mod = self.lookahead_distance
        self.lookahead_distance = lookahead_mod
        fov = np.pi/6.

        # All points observed?
        if np.count_nonzero(~self.observed_mask) == 0:
            return (np.ones((self.depth,)) * np.pi/6, 0, self.observed_mask.copy())

        while not best_combination_found:
            best_combination = (None, 0, self.observed_mask.copy())  # Sequence of commands, cumulated value, observed points
            original_observed_mask = self.observed_mask.copy()
            best_heuristic_debug = None

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
                    for n, state in enumerate(virtual_uav.run(self.time_step,
                                                              int(self.command_execution_time / self.time_step))):
                        # Observe only when the UAV is horizontal
                        if n % 2 and np.isclose(virtual_uav.state[3], 0.):
                            closeto = self.close_to(state[0:2], self.observation_radius, None)
                            observed_mask |= closeto
                if input_sequence[0] == 0. and input_sequence[1] == 0.:
                    print()
                # Compute how many points we observed during this input sequence
                visited_value = np.count_nonzero(~original_observed_mask&observed_mask)
                # Estimate how many points could be reached in the next search
                # (those in front of the uav and closer than a threshold, already visited discarded)
                heuristic_value, points_heuristic, p_infront, p_close = self.estimate_heuristic(
                    virtual_uav.state, fov, observed_mask)
                print("{}".format(input_sequence, visited_value, heuristic_value))


                # If the result is improved, save it
                cumulated_value = visited_value + heuristic_value
                if (cumulated_value) > best_combination[1]:
                    best_combination = (input_sequence, cumulated_value, observed_mask)
                    best_heuristic_debug = (heuristic_value, points_heuristic, p_infront, p_close)
                    logging.debug("Found a better combination: {}\n\t value: {}\n\t n visited: {}".format(
                                 str(best_combination[0]), best_combination[1], np.count_nonzero(best_combination[2])))

            # Once all combinations were tested...
            if best_combination[0] is None:
                best_combination_found = True
                dest = self.points[~self.observed_mask][np.argmin(cdist(self.points[~self.observed_mask], [virtual_uav.state[0:2]]))] - virtual_uav.state[0:2]
                angle = wrap_angle_rad(np.arctan2(dest[1], dest[0]) - (wrap_angle_rad(virtual_uav.state[2]) - np.pi/2))
                n_turns = np.clip(int(np.round(angle / (np.pi/6))), -self.depth, self.depth)
                inputs = list(itertools.repeat(np.pi / 6 * -np.sign(n_turns), np.abs(n_turns)))
                inputs.extend(list(itertools.repeat(0., self.depth - np.abs(n_turns))))
                best_combination = (np.array(inputs), 0, self.observed_mask.copy())
                # logging.info("No suitable input combination. Increasing lookahead to {}".format(
                #     self.lookahead_distance))
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
    import matplotlib.cm
    from fire_rs.geodata import environment
    import fire_rs.firemodel.propagation as propagation
    import fire_rs.planning.observation_path_search
    import fire_rs.geodata.geo_data
    import os

    z_order = {'uav':1000, 'observations':500, 'firefront':1}

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
        for iterations in range(100):
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
                plt.plot(local_traj[..., 0], local_traj[..., 1], 'darkgray', linewidth=2, zorder=z_order['uav']-1)
            if visited_points is not None:
                plt.plot(visited_points[:, 0], visited_points[:, 1], 'o', markerfacecolor=color_observed,
                         markeredgecolor=color_observed, markersize=3, zorder=z_order['observations'])
                plot_uav(plt.gca(), uav.state, size=5, zorder=z_order['uav'])
                plt.show()

    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    area = [[510000.0, 520000.0], [6200000.0, 6210000.0]]

    ignition_point = (200, 200)
    area_wind = (10, np.pi)
    ignition_times = None
    if not os.path.exists('/tmp/ignition.tif'):
        ignition_times = burn(area, ignition_point, area_wind)
        ignition_times.write_to_file('/tmp/ignition.tif')
    else:
        ignition_times = fire_rs.geodata.geo_data.GeoData.load_from_file('/tmp/ignition.tif')

    world = environment.World()
    some_area = world.get_elevation(area)

    uav = FixedWing(15., np.pi / 6, initial_state=np.array([515000.0, 6205000.0, np.pi/3, 0.]))
    uav.input = np.array([0., ])  # [x, y, ψ, ϕ]

    # draw UAV
    plot_uav(plt.gca(), uav.state, size=5)
    some_area.plot()
    ignition_times.plot(cmap=matplotlib.cm.Reds)

    n = 0
    colors_burning = ['red', 'yellow']
    colors_observed = ['darkred', 'orange']
    t_start, t_end, t_step = 75, 240, 1
    for low, high in zip(range(t_start, t_end-t_step, t_step), range(t_start+t_step, t_end, t_step)):
        ignition_array = ignition_times.data['ignition'] / 60


        selected_points = np.array(list(zip(*np.where((low < ignition_array) & (ignition_array < high)))), dtype=np.float)
        selected_points *= np.array([ignition_times.cell_width, ignition_times.cell_height], dtype=np.float)
        selected_points += np.array([ignition_times.x_offset, ignition_times.y_offset], dtype=np.float)

        try:

            plt.plot(selected_points[..., 0], selected_points[..., 1], 'o', markerfacecolor=colors_burning[n % 2],
                         markeredgecolor=colors_burning[n % 2], markersize=5, zorder=z_order['firefront'])
            observe(selected_points, uav, colors_observed[n % 2])
            n += 1
            plt.show()
        except ValueError:
            logging.exception("")


if __name__ == '__main__':
    demo_firefornt_observation()
    plt.show()
