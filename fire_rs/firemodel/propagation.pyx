# cython: profile=True

# Provides a simple binary heap. An optimized binary heap with update capabilities is available at:
# https://mail.scipy.org/pipermail/scipy-user/2009-December/023539.html
import heapq

import numpy as np

import fireshapes
import rothermel


class Environment:
    def __init__(self, max_x: int, max_y: int, cell_size: int):
        """Abstract class providing access to the main properties of the environment

        :param max_x: Number of cells on x axis
        :param max_y: Number of cells on y axis
        :param cell_size: Size [m] of the side of a cell
        """
        self.max_x = max_x
        self.max_y = max_y
        self.cell_size = cell_size

    def get_fuel_type(self, x, y):
        """Returns the fuel type (e.g. 'SH5') in (x,y)"""
        return

    def get_wind(self, x, y):
        """Returns a tuple (wind_speed [km/h], wind_angle [rad]) in (x,y)"""
        return

    def get_moisture(self, x, y):
        """Returns the moisture scenario (e.g. 'D1L3') in (x,y)"""
        return

    def get_elevation(self, x, y, default=None):
        """Returns the elevation [m] in (x,y)"""
        return default

    def get_slope(self, x, y):
        """Computes the slope of a given point.
        http://desktop.arcgis.com/fr/arcmap/10.3/tools/spatial-analyst-toolbox/how-slope-works.htm

        :returns: The slope [percent] in (x, y) and the direction in which the slope raises most.
        """
        e = self.get_elevation(x, y)
        a = self.get_elevation(x - 1, y + 1, default=e)
        b = self.get_elevation(x, y + 1, default=e)
        c = self.get_elevation(x + 1, y + 1, default=e)
        d = self.get_elevation(x - 1, y, default=e)
        f = self.get_elevation(x + 1, y, default=e)
        g = self.get_elevation(x - 1, y - 1, default=e)
        h = self.get_elevation(x, y - 1, default=e)
        i = self.get_elevation(x + 1, y - 1, default=e)
        dzdx = (c + 2*f + i - a - 2*d - g) / 8 * self.cell_size
        dzdy = (g + 2*h + i - a - 2*b - c) / 8 * self.cell_size
        rise_run = np.sqrt(np.power(dzdx, 2) + np.power(dzdy, 2))
        slope_percent = rise_run * 100
        raise_dir = np.arctan2(dzdy, dzdx)
        return slope_percent, raise_dir

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


class DummyEnvironment(Environment):
    """A sample environment for test purposes. Associates default values to each environment parameter."""

    def get_fuel_type(self, x, y):
        return 'SH6'

    def get_wind(self, x, y):
        return 10, np.pi * (x / self.max_x)  # speed [km/h], angle [rad]

    def get_moisture(self, x, y):
        return 'D2L2'

    def get_elevation(self, x, y, default=None):
        return 100  # elevation in meters


# graph connectivity: each element is a (delta-x, delta-y)
neighborhood = [(1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1),
    (2,1), (2,-1), (-2,1), (-2,-1), (1,2), (1,-2), (-1,2), (-1,-2)]



def propagate(env: Environment, x: int, y: int):
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param x: X coordinate of ignition point
    :param y: Y coordinate of ignition point
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    # ignition times of each cell, infinite before propagation
    ignition_times = 2**63-1 * np.ones((env.max_x, env.max_y))

    # select ignition point and add to queue
    ignition = (0, (x, y))  # (ignition-time, (x, y))
    ignition_times[ignition[1]] = ignition[0]
    q = [ignition]  # queue of labels (t, (x,y)) indicating an upper bound on the ignition time 't' of a cell '(x, y)'
    heapq.heapify(q)

    # Dijkstra propagation of fire
    while not len(q) == 0:
        # select current point
        (t, (x, y)) = heapq.heappop(q)
        # neighbors in the grid
        accessible_neighbors = [(dx, dy) for (dx, dy) in neighborhood if
                                0 <= x + dx < env.max_x and 0 <= y + dy < env.max_y]
        spread_shape = env.get_propagation_shape(x, y)
        # for each neighbor, propagate earliest ignition time
        for (dx, dy) in accessible_neighbors:
            dist = np.sqrt(dx * dx + dy * dy) * env.cell_size  # dist from current to neighbor
            angle = np.arctan2(dy, dx)  # angle to neighbor
            speed = spread_shape.speed(angle)  # ros in the direction of neighbor
            dt = dist / speed  # time for fire to reach neighbor
            if ignition_times[x + dx, y + dy] > t + dt:
                ignition_times[x + dx, y + dy] = t + dt
                heapq.heappush(q, (t + dt, (x + dx, y + dy)))
    return ignition_times

if __name__ == '__main__':
    env = DummyEnvironment(50, 50, 10)
    print(propagate(env, 5, 5))
