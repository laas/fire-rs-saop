# cython: profile=True

# Provides a simple binary heap. An optimized binary heap with update capabilities is available at:
# https://mail.scipy.org/pipermail/scipy-user/2009-December/023539.html
import heapq

import numpy as np

from fire_rs.geodata.clustering import cluster_multi_layer
from fire_rs.geodata.environment import World
from fire_rs.geodata.geo_data import GeoData

import fire_rs.firemodel.fireshapes as fireshapes
import fire_rs.firemodel.rothermel as rothermel
import fire_rs.firemodel.environment as env


class Environment:
    def __init__(self, area, wind_speed, wind_dir):
        """Abstract class providing access to the main properties of the environment

        :param area: ((x_min, x_max), y_min, y_max))
        """
        world = World()
        slope = world.get_slope(area)
        wind = world.get_wind(area, domain_average=(wind_speed, wind_dir))
        moisture = slope.clone(fill_value=env.get_moisture_scenario_id('D1L1'), dtype=[('moisture', 'int32')])
        fuel = slope.clone(fill_value=env.get_fuel_model_id('SH5'), dtype=[('fuel', 'int32')])
        self.raster = slope.combine(wind).combine(moisture).combine(fuel)
        self._clustering = None

    @property
    def clustering(self):
        if self._clustering is None:
            base_for_clustering = self.raster.slice(['wind_angle', 'wind_velocity', 'fuel', 'moisture'])
            self._clustering = cluster_multi_layer(base_for_clustering,
                                                   err_by_layer={'wind_angle': 0.2, 'wind_velocity': 2},
                                                   already_clustered=['fuel', 'moisture'])
        return self._clustering

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
        slope_equivalent = summary.equivalent_slope
        x_w_eff = wind_speed * np.cos(wind_dir) + slope_equivalent * np.cos(slope_dir)
        y_w_eff = wind_speed * np.sin(wind_dir) + slope_equivalent * np.sin(slope_dir)
        angle_w_eff = np.arctan2(y_w_eff, x_w_eff)
        speed_w_eff = np.sqrt(x_w_eff**2 + y_w_eff**2)
        return ros, angle_w_eff, speed_w_eff

    def get_propagation_shape(self, x, y):
        ros, wind_angle, wind_speed = self.get_spread_parameters(x, y)
        return fireshapes.get_fire_shape(wind_speed, wind_angle, ros)


# graph connectivity: each element is a (delta-x, delta-y)
neighborhood = [(1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1),
    (2,1), (2,-1), (-2,1), (-2,-1), (1,2), (1,-2), (-1,2), (-1,-2)]


class FirePropagation(GeoData):
    """Class to compute and store fire propagation data."""

    def __init__(self, environment: 'Environment'):
        self.environment = environment
        # build internal data structure compose of three layers ['ignition', 'x_pred', 'y_pred']
        tmp = environment.raster.clone(fill_value=2**63-1, dtype=[('ignition', 'float32')])
        tmp2 = environment.raster.clone(fill_value=-1, dtype=[('x_pred', 'int32'), ('y_pred', 'int32')])
        super().__init__(tmp.combine(tmp2).data, environment.raster.x_offset, environment.raster.y_offset,
                         environment.raster.cell_width, environment.raster.cell_height)

        self._propagation_queue = []
        heapq.heapify(self._propagation_queue)
        (self.max_x, self.max_y) = self.data.shape

    def set_ignition_point(self, x, y, time):
        """Sets the given (x, y) point as on fire at the given time.
        Multiple ignition points can be provided.
        """
        # ignition time of (x, y) is time
        self.data[x, y][0] = time
        # predecessor of (x, y) is itself (i.e. meaning no predecessor)
        self.data[x, y][1] = x
        self.data[x, y][2] = y
        self._push_to_propagation_queue(x, y, time)

    def _push_to_propagation_queue(self, x, y, time):
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

    def propagate(self, horizon):
        assert self.cell_width == self.cell_height
        cell_size = self.cell_height
        # Dijkstra propagation of fire
        while not len(self._propagation_queue) == 0:
            # peek top value
            t, __ = self._pick_from_propagation_queue()
            if t >= horizon:
                break
            # select current point
            (t, (x, y)) = self._pop_from_propagation_queue()
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
                if self.data[x + dx, y + dy][0] > t + dt:
                    # update ignition time, predecessor and add to queue
                    self.data[x + dx, y + dy][0] = t + dt
                    self.data[x + dx, y + dy][1] = x
                    self.data[x + dx, y + dy][2] = y
                    self._push_to_propagation_queue(x + dx, y + dy, t + dt)


def propagate(env: Environment, x: int, y: int) -> 'FirePropagation':
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param x: X coordinate of ignition point
    :param y: Y coordinate of ignition point
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    fp = FirePropagation(env)
    fp.set_ignition_point(x, y, 0)
    fp.propagate(horizon=np.inf)
    return fp


if __name__ == '__main__':
    pass