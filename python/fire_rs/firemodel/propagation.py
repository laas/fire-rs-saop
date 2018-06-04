# cython: profile=True

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

# Provides a simple binary heap. An optimized binary heap with update capabilities is available at:
# https://mail.scipy.org/pipermail/scipy-user/2009-December/023539.html
import heapq
import logging

from typing import List, Tuple, Union

from collections import Sequence

import numpy as np

from fire_rs.geodata.clustering import cluster_multi_layer
from fire_rs.geodata.environment import World
from fire_rs.geodata.geo_data import GeoData, Cell, TimedPoint

import fire_rs.firemodel.fireshapes as fireshapes
import fire_rs.firemodel.rothermel as rothermel
import fire_rs.firemodel.environment as env

from fire_rs.deprecation import deprecated

logger = logging.getLogger(__name__)


class Environment:
    def __init__(self, area, wind_speed, wind_dir):
        """Abstract class providing access to the main properties of the environment

        :param area: ((x_min, x_max), (y_min, y_max))
        :param wind_speed: mean wind speed in km/h
        :param wind_dir: mean wind direction in radians
        """
        self._wind_speed = wind_speed  # type: float
        self._wind_dir = wind_dir  # type: float
        self._area = area  # type: ((float, float), (float, float))
        self._world = World()  # type: World
        elevation = self._world.get_elevation(area)
        slope = self._world.get_slope(area)
        wind = self._world.get_wind(area, domain_average=(wind_speed, wind_dir))
        moisture = slope.clone(fill_value=env.get_moisture_scenario_id('D1L1'),
                               dtype=[('moisture', 'int32')])
        fuel = self._world.get_fuel_type(area)
        # type: GeoData
        self.raster = slope.combine(wind).combine(moisture).combine(fuel).combine(elevation)
        self._clustering = None

    @property
    def area(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        return self._area

    @property
    @deprecated
    def clustering(self):
        if self._clustering is None:
            base_for_clustering = self.raster.slice(
                ['wind_angle', 'wind_velocity', 'fuel', 'moisture'])
            self._clustering = cluster_multi_layer(base_for_clustering,
                                                   err_by_layer={'wind_angle': 0.2,
                                                                 'wind_velocity': 2},
                                                   already_clustered=['fuel', 'moisture'])
        return self._clustering

    def update_area_wind(self, wind_speed, wind_dir):
        new_wind = self._world.get_wind(self._area, domain_average=(wind_speed, wind_dir))
        self.raster.data['wind_velocity'] = new_wind['wind_velocity']
        self.raster.data['wind_angle'] = new_wind['wind_angle']

    def get_fuel_type(self, x, y):
        """Returns the fuel type (e.g. 'SH5') in (x,y)"""
        return env.get_fuel_model_name(int(self.raster.data[x, y]['fuel']))

    def get_wind(self, x, y):
        """Returns a tuple (wind_speed [km/h], wind_angle [rad]) in (x,y)"""
        tmp = self.raster.data[x, y]
        wind_vel = tmp['wind_velocity']
        wind_angle = tmp['wind_angle']
        return wind_vel, wind_angle

    def get_moisture(self, x, y):
        """Returns the moisture scenario (e.g. 'D1L3') in (x,y)"""
        return env.get_moisture_scenario_name(int(self.raster.data[x, y]['moisture']))

    def get_slope(self, x, y):
        tmp = self.raster.data[x, y]
        return tmp['slope'], tmp['raise_dir']

    def get_spread_parameters(self, x, y):
        """Computes the three parameters dictating the spread of the fire.
         - ros: Rate of Spread [m/s] in main direction
         - effective wind angle: main direction of the fire spread, accounting for wind and slope
         - effective wind speed: wind equivalent [km/h] that takes into account wind and slope
        """
        fuel_type = self.get_fuel_type(x, y)
        moisture = self.get_moisture(x, y)
        slope_percent, slope_dir = self.get_slope(x, y)
        wind_speed, wind_dir = self.get_wind(x, y)
        summary = rothermel.ros(fuel_type, moisture, wind_speed, slope_percent)
        ros = summary.ros
        if np.isnan(ros):
            logger.warning(
                "RoS is NaN from fuel_type:%s moisture:%s slope:%s wind_speed:%s wind_dir:%s",
                fuel_type, moisture, slope_percent, wind_speed, wind_dir)
        slope_equivalent = summary.equivalent_slope
        x_w_eff = wind_speed * np.cos(wind_dir) + slope_equivalent * np.cos(slope_dir)
        y_w_eff = wind_speed * np.sin(wind_dir) + slope_equivalent * np.sin(slope_dir)
        angle_w_eff = np.arctan2(y_w_eff, x_w_eff)
        speed_w_eff = np.sqrt(x_w_eff ** 2 + y_w_eff ** 2)
        return ros, angle_w_eff, speed_w_eff

    def get_propagation_shape(self, x, y):
        ros, wind_angle, wind_speed = self.get_spread_parameters(x, y)
        return fireshapes.get_fire_shape(wind_speed, wind_angle, ros)


# graph connectivity: each element is a (delta-x, delta-y)
neighborhood = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1),
                (2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (1, -2), (-1, 2), (-1, -2)]


class FirePropagation:
    """Class to compute and store fire propagation data."""

    def __init__(self, environment: 'Environment', ignition_layer='ignition'):
        self.environment = environment
        self._ignition_layer = ignition_layer
        # build internal data structure compose of three layers ['ignition', 'x_pred', 'y_pred']
        self.prop_data = environment.raster.clone(fill_value=np.finfo(np.float64).max,
                                                  dtype=[(self._ignition_layer, 'float64')])
        tmp2 = environment.raster.clone(fill_value=-1, dtype=[('x_pred', 'int32'),
                                                              ('y_pred', 'int32')])
        self.prop_data = self.prop_data.combine(tmp2)

        self._propagation_queue = []  # type: List[float, Tuple[int, int]]
        heapq.heapify(self._propagation_queue)
        (self.max_x, self.max_y) = self.prop_data.data.shape
        self._ignition_cells = []  # type: List[Tuple[int, int]]

    def set_ignition_point(self, ignition_point: Union[TimedPoint, Tuple[float, float, float]]):
        """Sets the given (x, y) point as on fire at the given time.
        Multiple ignition points can be provided.
        """
        (xi, yi) = self.environment.raster.array_index((ignition_point[0], ignition_point[1]))
        self.set_ignition_cell((xi, yi, ignition_point[2]))

    def set_ignition_cell(self, ign_cell: Tuple[int, int, float]):
        """Sets the given (x, y) cells as on fire at the given time t."""
        self.prop_data.data[ign_cell[0], ign_cell[1]][self._ignition_layer] = ign_cell[2]
        # predecessor of (x, y) is itself (i.e. meaning no predecessor)
        self.prop_data.data[ign_cell[0], ign_cell[1]]['x_pred'] = ign_cell[0]
        self.prop_data.data[ign_cell[0], ign_cell[1]]['y_pred'] = ign_cell[1]
        self._push_to_propagation_queue(ign_cell[0], ign_cell[1], ign_cell[2])
        self._ignition_cells.append((ign_cell[0], ign_cell[1]))

    def ignitions(self) -> GeoData:
        return self.prop_data.slice([self._ignition_layer])

    def _push_to_propagation_queue(self, x: int, y: int, time: float):
        heapq.heappush(self._propagation_queue, (time, (x, y)))

    def _pick_from_propagation_queue(self):
        assert len(self._propagation_queue) > 0
        return self._propagation_queue[0]

    def _pop_from_propagation_queue(self):
        assert len(self._propagation_queue) > 0
        return heapq.heappop(self._propagation_queue)

    @property
    def propagation_finished(self):
        return len(self._propagation_queue) == 0

    def propagate(self, horizon: float):
        assert self.prop_data.cell_width == self.prop_data.cell_height
        cell_size = self.prop_data.cell_height

        d = self.prop_data.data

        # Assert the ignition point can burn
        t, (x, y) = self._pick_from_propagation_queue()
        igni_point_fuel_type = self.environment.get_fuel_type(x, y)
        if igni_point_fuel_type[:2] == "NB":
            logger.warning(
                "Ignition point %s is set in a nonburnable cell (%s). Fire won't propagate.",
                str((x, y)), str(igni_point_fuel_type))

        # Dijkstra propagation of fire
        while not len(self._propagation_queue) == 0:
            # peek top value
            t, __ = self._pick_from_propagation_queue()
            if t >= horizon:
                break
            # select current point
            (t, (x, y)) = self._pop_from_propagation_queue()
            neighbor_fuel_type = self.environment.get_fuel_type(x, y)
            if neighbor_fuel_type[:2] == "NB":
                continue
            # neighbors in the grid
            accessible_neighbors = [(dx, dy) for (dx, dy) in neighborhood
                                    if 0 <= x + dx < self.max_x and 0 <= y + dy < self.max_y]
            spread_shape = self.environment.get_propagation_shape(x, y)
            # for each neighbor, propagate earliest ignition time
            for (dx, dy) in accessible_neighbors:
                dist = np.sqrt(dx * dx + dy * dy) * cell_size  # dist from current to neighbor
                angle = np.arctan2(dy, dx)  # angle to neighbor
                speed = spread_shape.speed(angle)  # ros in the direction of neighbor
                dt = dist / speed  # time for fire to reach neighbor
                if d[x + dx, y + dy][0] > t + dt:
                    # update ignition time, predecessor and add to queue
                    d[x + dx, y + dy][0] = t + dt
                    d[x + dx, y + dy][1] = x
                    d[x + dx, y + dy][2] = y
                    self._push_to_propagation_queue(x + dx, y + dy, t + dt)

    @deprecated
    def information_matrix(self):
        d = self.prop_data.clone(fill_value=0,
                                 dtype=[('env_cluster', 'int32'), ('propagation_angle', 'float64'),
                                        ('propagation_speed', 'float64'),
                                        ('error_on_speed', 'float64')])
        for x in range(0, d.max_x):
            for y in range(0, d.max_y):
                dxy = d.data[x, y]
                cluster, angle, speed, error = self.information_gain(x, y)
                dxy[0] = cluster
                dxy[1] = angle
                dxy[2] = speed
                dxy[3] = error

        c = d.slice(['env_cluster', 'propagation_angle'])
        cc = cluster_multi_layer(c, {'propagation_angle': 0.1}, already_clustered=['env_cluster'],
                                 cluster_layer_name='information_clustering')
        return d.combine(cc)

    @deprecated
    def information_gain(self, x, y):
        def pred(x, y):
            return self.prop_data.data[x, y][1], self.prop_data.data[x, y][2]

        def has_pred(x, y):
            return pred(x, y) != (x, y) and pred(x, y) != (-1, -1) and \
                   cluster_of(x, y) == cluster_of(*pred(x, y))

        def propagation_angle(x, y):
            if pred(x, y) != (x, y) and pred(x, y) != (-1, -1):
                px, py = pred(x, y)
                return np.arctan2(y - py, x - px)
            else:
                return 0

        def cluster_of(x, y):
            return self.environment.clustering.data[x, y][0]

        def ignition_time(x, y):
            return self.prop_data.data[x, y][0]

        def oldest_straight_line_pred(x, y):
            if not has_pred(x, y) or cluster_of(x, y) != cluster_of(*pred(x, y)):
                return -1, -1
            px, py = pred(x, y)
            while has_pred(px, py) and propagation_angle(px, py) == propagation_angle(x, y) \
                    and cluster_of(x, y) == cluster_of(*pred(px, py)):
                px, py = pred(px, py)
            return px, py

        if has_pred(x, y):
            px, py = oldest_straight_line_pred(x, y)
            dist = np.sqrt((x - px) ** 2 + (y - py) ** 2) * self.prop_data.cell_height
            ts = ignition_time(*pred(x, y))
            te = ignition_time(x, y)
            speed = dist / (te - ts)
            traversal_time = self.prop_data.cell_height / speed
            min_obs_speed = dist / (te + traversal_time - ts)
            max_obs_speed = dist / max(0.00001, (te - (ts + traversal_time)))
            error = max_obs_speed - min_obs_speed

            return cluster_of(x, y), propagation_angle(x, y), speed, error
        else:
            return cluster_of(x, y), propagation_angle(x, y), np.nan, np.nan


def propagate_from_points(env: Environment, ignitions_points: Union[TimedPoint, List[TimedPoint]],
                          horizon: float = np.inf) -> FirePropagation:
    """Simulate a fire from all ignition points until the 
    
    :param env: Environment model.
    :param ignitions_points: Fire start points, giving for each one (x,y) coordinates and fire start time.
    :param horizon: Absolute time at which the propagation stops.
    :return: 
    """
    fp = FirePropagation(env)
    # If ignitions_points is a sequence of sequences, then it is a list of points
    if isinstance(ignitions_points[0], Sequence):
        for tp in ignitions_points:
            fp.set_ignition_point(tp)
    else:
        fp.set_ignition_point(ignitions_points)
    fp.propagate(horizon=horizon)
    return fp


def propagate_from_cell(env: Environment, cell: Cell, horizon=np.inf) -> 'FirePropagation':
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param cell: (x,y) coordinates of the ignition point (as array index)
    :param horizon: Wall-clock time at which to stop the propagation
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    fp = FirePropagation(env)
    pt = env.raster.coordinates(cell)
    fp.set_ignition_point(TimedPoint(pt.x, pt.y, 0))
    fp.propagate(horizon=horizon)
    return fp


if __name__ == '__main__':
    pass
