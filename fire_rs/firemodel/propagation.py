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

        :param area: ((x_min, x_max), (y_min, y_max))
        """
        world = World()
        elevation = world.get_elevation(area)
        slope = world.get_slope(area)
        wind = world.get_wind(area, domain_average=(wind_speed, wind_dir))
        moisture = slope.clone(fill_value=env.get_moisture_scenario_id('D1L1'), dtype=[('moisture', 'int32')])
        fuel = world.get_fuel_type(area)
        self.raster = slope.combine(wind).combine(moisture).combine(fuel).combine(elevation)
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
        if np.isnan(ros):
            print("WARNING: RoS is NaN from fuel_type:{} moisture:{} slope:{} wind_speed:{} wind_dir:{}",
                  fuel_type, moisture, slope_percent, wind_speed, wind_dir)
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


class FirePropagation:
    """Class to compute and store fire propagation data."""

    def __init__(self, environment: 'Environment'):
        self.environment = environment
        # build internal data structure compose of three layers ['ignition', 'x_pred', 'y_pred']
        self.prop_data = environment.raster.clone(fill_value=2**63-1, dtype=[('ignition', 'float32')])
        tmp2 = environment.raster.clone(fill_value=-1, dtype=[('x_pred', 'int32'), ('y_pred', 'int32')])
        self.prop_data = self.prop_data.combine(tmp2)

        self._propagation_queue = []
        heapq.heapify(self._propagation_queue)
        (self.max_x, self.max_y) = self.prop_data.data.shape
        self._ignition_points = []

    def set_ignition_point(self, x, y, time):
        """Sets the given (x, y) point as on fire at the given time.
        Multiple ignition points can be provided.
        """
        # ignition time of (x, y) is time
        self.prop_data.data[x, y][0] = time
        # predecessor of (x, y) is itself (i.e. meaning no predecessor)
        self.prop_data.data[x, y][1] = x
        self.prop_data.data[x, y][2] = y
        self._push_to_propagation_queue(x, y, time)
        self._ignition_points.append((x, y))

    def ignitions(self) -> GeoData:
        return self.prop_data.slice(['ignition'])

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
        assert self.prop_data.cell_width == self.prop_data.cell_height
        cell_size = self.prop_data.cell_height

        d = self.prop_data.data

        # Assert the ignition point can burn
        t, (x, y) = self._pick_from_propagation_queue()
        assert self.environment.get_fuel_type(x, y)[:2] != "NB"

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

    def information_matrix(self):
        d = self.prop_data.clone(fill_value=0, dtype=[('env_cluster', 'int32'), ('propagation_angle', 'float32'),
                                                      ('propagation_speed', 'float32'), ('error_on_speed', 'float32')])
        for x in range(0, d.max_x):
            for y in range(0, d.max_y):
                dxy = d.data[x, y]
                cluster, angle, speed, error = self.information_gain(x, y)
                dxy[0] = cluster
                dxy[1] = angle
                dxy[2] = speed
                dxy[3] = error

        c = d.slice(['env_cluster', 'propagation_angle'])
        cc = cluster_multi_layer(c, {'propagation_angle': 0.1}, already_clustered=['env_cluster'], cluster_layer_name='information_clustering')
        return d.combine(cc)

    def information_gain(self, x, y):
        def pred(x, y):
            return self.prop_data.data[x, y][1], self.prop_data.data[x, y][2]

        def has_pred(x, y):
            return pred(x, y) != (x, y) and pred(x, y) != (-1, -1) and cluster_of(x, y) == cluster_of(*pred(x, y))

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
            while has_pred(px, py) and propagation_angle(px, py) == propagation_angle(x, y)\
                    and cluster_of(x, y) == cluster_of(*pred(px, py)):
                px, py = pred(px, py)
            return px, py

        if has_pred(x, y):
            px, py = oldest_straight_line_pred(x, y)
            dist = np.sqrt((x-px)**2 + (y-py)**2) * self.prop_data.cell_height
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

    def plot(self, axes=None, blocking=False):
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.colors import LightSource
        from matplotlib import cm

        world = self.environment.raster

        x = np.arange(world.max_x)
        x = (x * world.cell_width) + world.x_offset

        y = np.arange(world.max_y)
        y = (y * world.cell_height) + world.y_offset

        X, Y = np.meshgrid(x, y)
        Z = world['elevation']

        def plot_elevation_shade(ax, x, y, z):
            cbar_lim = (z.min(), z.max())

            image_scale = (x[0][0], x[0][-1], y[0][0], y[-1][0])

            ls = LightSource(azdeg=315, altdeg=45)
            ax.imshow(ls.hillshade(z, vert_exag=5, dx=world.cell_width, dy=world.cell_width),
                      extent=image_scale, cmap='gray')

            return ax.imshow(ls.shade(z, cmap=cm.terrain, blend_mode='overlay', vert_exag=1,
                                      dx=world.cell_width, dy=world.cell_width,
                                      vmin=cbar_lim[0], vmax=cbar_lim[1]),
                             extent=image_scale, vmin=cbar_lim[0], vmax=cbar_lim[1], cmap=cm.terrain)

        def plot_wind_arrows(ax, x, y, wx, wy):
            ax.quiver(x, y, wx, wy, pivot='middle', color='dimgrey')

        if axes:
            fire_ax = axes
            fire_fig = None
        else:
            fire_fig = plt.figure()
            fire_ax = fire_fig.gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

        ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
        fire_ax.yaxis.set_major_formatter(ax_formatter)
        fire_ax.xaxis.set_major_formatter(ax_formatter)
        shade = plot_elevation_shade(fire_ax, X, Y, Z.T[::-1, ...])
        if fire_fig:
            cbar = fire_fig.colorbar(shade, shrink=0.5, aspect=2)
            cbar.set_label("Height [m]")

        wind_vel = world['wind_velocity']
        wind_ang = world['wind_angle']
        WX = wind_vel * np.cos(wind_ang)
        WY = wind_vel * np.sin(wind_ang)

        plot_wind_arrows(fire_ax, *np.meshgrid(x[::10], y[::10]), WX[::10, ::10], WY[::10, ::10])

        # mask all cells whose ignition has not been computed
        igni = np.array(self.ignitions().data['ignition'])
        igni[igni >= 99999999999] = np.nan

        # plot fire front with contour lines in minutes
        fronts = fire_ax.contour(x, y, igni.T / 60, 10, cmap=cm.Set1)
        fire_ax.clabel(fronts, inline=True, fontsize='smaller', inline_spacing=1, linewidth=2, fmt='%.0f')

        for ignition_point in self._ignition_points:
            coords = self.prop_data.coordinates(ignition_point)
            fire_ax.plot(*coords, 'or', linewidth=5)

        plt.show(block=blocking)
        return fire_ax


def propagate(env: Environment, x: int, y: int, horizon=np.inf) -> 'FirePropagation':
    """Set the environment on fire in (x, y) at time 0 and returns the ignition times of other points of the environment.

    :param env: Environment model
    :param x: X coordinate of ignition point
    :param y: Y coordinate of ignition point
    :param horizon: Wall-clock time at which to stop the propagation
    :return: A matrix of ignition time. The size of the matrix is given by the size of the environment
    """
    fp = FirePropagation(env)
    fp.set_ignition_point(x, y, 0)
    fp.propagate(horizon=horizon)
    return fp


if __name__ == '__main__':
    pass