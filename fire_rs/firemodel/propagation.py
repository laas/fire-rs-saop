# cython: profile=True

# Provides a simple binary heap. An optimized binary heap with update capabilities is available at:
# https://mail.scipy.org/pipermail/scipy-user/2009-December/023539.html
import heapq

import numpy as np

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


def propagate(env: Environment, x: int, y: int) -> 'GeoData':
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param x: X coordinate of ignition point
    :param y: Y coordinate of ignition point
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    return next(propagator(env, x, y, yield_tstep=np.inf))


def propagator(env: Environment, x: int, y: int, yield_tstep=np.inf) -> 'GeoData':
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param x: X coordinate of ignition point
    :param y: Y coordinate of ignition point
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    # ignition times of each cell, infinite before propagation
    ignition_times_geo = env.raster.clone(fill_value=2**63-1, dtype=[('ignition', 'float32')])
    ignition_times = ignition_times_geo.data.view('float32')
    (max_x, max_y) = ignition_times.shape
    assert ignition_times_geo.cell_width == ignition_times_geo.cell_height
    cell_size = ignition_times_geo.cell_height

    t = 0

    # select ignition point and add to queue
    ignition = (t, (x, y))  # (ignition-time, (x, y))
    ignition_times[ignition[1]] = ignition[0]
    q = [ignition]  # queue of labels (t, (x,y)) indicating an upper bound on the ignition time 't' of a cell '(x, y)'
    heapq.heapify(q)

    # yielding frequency
    last_yield = yield_tstep

    # Dijkstra propagation of fire
    while not len(q) == 0:
        # peek top value
        t, __ = q[0]
        if t >= last_yield:
            last_yield += yield_tstep
            yield ignition_times_geo
        # select current point
        (t, (x, y)) = heapq.heappop(q)
        # neighbors in the grid
        accessible_neighbors = [(dx, dy) for (dx, dy) in neighborhood if
                                0 <= x + dx < max_x and 0 <= y + dy < max_y]
        spread_shape = env.get_propagation_shape(x, y)
        # for each neighbor, propagate earliest ignition time
        for (dx, dy) in accessible_neighbors:
            dist = np.sqrt(dx * dx + dy * dy) * cell_size  # dist from current to neighbor
            angle = np.arctan2(dy, dx)  # angle to neighbor
            speed = spread_shape.speed(angle)  # ros in the direction of neighbor
            dt = dist / speed  # time for fire to reach neighbor
            if ignition_times[x + dx, y + dy] > t + dt:
                ignition_times[x + dx, y + dy] = t + dt
                heapq.heappush(q, (t + dt, (x + dx, y + dy)))
    yield ignition_times_geo


if __name__ == '__main__':
    pass