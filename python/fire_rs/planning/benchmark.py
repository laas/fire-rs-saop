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

import logging
import matplotlib
matplotlib.use('Agg')  # do not require X display to plot figures that are not shown
import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot
import numpy as np
import random
import time
import types

from collections import namedtuple, Sequence
from itertools import cycle
from typing import Optional, Tuple, Union

import fire_rs.geodata.display
import fire_rs.uav_planning as up

from fire_rs.geodata.geo_data import TimedPoint, Area
from fire_rs.geodata.geo_data import Point as GeoData_Point


_DBL_MAX = np.finfo(np.float64).max

DISCRETE_ELEVATION_INTERVAL = 100

Waypoint = namedtuple('Waypoint', 'x, y, z, dir')


class UAV:

    def __init__(self, max_air_speed: float, max_angular_velocity: float, max_pitch_angle: float,
                 base_waypoint: 'Union(Waypoint, (float, float, float, float)]'):
        self.max_air_speed = max_air_speed
        self.max_angular_velocity = max_angular_velocity
        self.max_pitch_angle = max_pitch_angle
        self.base_waypoint = base_waypoint

    def as_cpp(self):
        return up.UAV(self.max_air_speed, self.max_angular_velocity, self.max_pitch_angle)

    def __repr__(self):
        return "".join(("UAV(max_air_speed=", repr(self.max_air_speed),
                        ", max_angular_velocity=", repr(self.max_angular_velocity),
                        ", base_waypoint=", repr(self.base_waypoint), ")"))


class Flight:

    def __init__(self, uav: UAV, start_time: float, max_flight_time: float):
        assert max_flight_time > 0
        assert start_time > 0
        self.uav = uav
        self.start_time = start_time
        self.max_flight_time = max_flight_time

    def as_trajectory_config(self):
        x = self.uav.base_waypoint
        wp = up.Waypoint(x[0], x[1], x[2], x[3])
        return up.TrajectoryConfig(self.uav.as_cpp(), wp, wp, self.start_time, self.max_flight_time)

    def __repr__(self):
        return "".join(("Flight(uav=", repr(self.uav), ", start_time=", repr(self.start_time),
                        ", max_flight_time=", repr(self.max_flight_time), ")"))


class Scenario:

    def __init__(self, area: ((float, float), (float, float)),
                 wind_speed: float,
                 wind_direction: float,
                 ignitions: [TimedPoint],
                 flights: [Flight]):
        self.area = area
        self.wind_speed = wind_speed
        self.wind_direction = wind_direction
        assert len(ignitions) > 0
        self.ignitions = ignitions
        assert len(flights) > 0
        self.flights = flights

        self.time_window_start = np.inf
        self.time_window_end = -np.inf
        for flight in flights:
            self.time_window_start = min(self.time_window_start, flight.start_time - 180)
            self.time_window_end = max(self.time_window_end, flight.start_time+flight.max_flight_time + 180)

    def __repr__(self):
        return "".join(["Scenario(area=", repr(self.area), ", wind_speed=", repr(self.wind_speed),
                        ", wind_direction=", repr(self.wind_direction), ", ignitions=", repr(self.ignitions),
                        ", flights=", repr(self.flights), ")"])


class PlanDisplayExtension(fire_rs.geodata.display.DisplayExtension):
    """Extension to GeoDataDisplay that an observation plan."""

    def __init__(self, plan_trajectory):
        self.plan_trajectory = plan_trajectory

    def extend(self, geodatadisplay):
        '''Bounds drawing methods to a GeoDataDisplayInstance.'''
        geodatadisplay.plan_trajectory = self.plan_trajectory
        geodatadisplay.draw_waypoints = types.MethodType(PlanDisplayExtension._draw_waypoints_extension, geodatadisplay)
        geodatadisplay.draw_flighttime_path = types.MethodType(PlanDisplayExtension._draw_flighttime_path_extension, geodatadisplay)
        geodatadisplay.draw_solid_path = types.MethodType(PlanDisplayExtension._draw_solid_path_extension, geodatadisplay)
        geodatadisplay.draw_segments = types.MethodType(PlanDisplayExtension._draw_segments_extension, geodatadisplay)
        geodatadisplay.draw_observedcells = types.MethodType(PlanDisplayExtension._draw_observedcells, geodatadisplay)

    def _draw_waypoints_extension(self, *args, **kwargs):
        '''Draw path waypoints in a GeoDataDisplay figure.'''
        color = kwargs.get('color', 'C0')
        waypoints = self.plan_trajectory.as_waypoints()
        x = [wp.x for wp in waypoints]
        y = [wp.y for wp in waypoints]
        self._drawings.append(self.axis.scatter(x[::2], y[::2], s=7, c=color, marker='D'))
        self._drawings.append(self.axis.scatter(x[1::2], y[1::2], s=7, c=color, marker='>'))

    def _draw_flighttime_path_extension(self, *args, colorbar_time_range: 'Optional[Tuple[float, float]]' = None,
                                        **kwargs):
        '''Draw trajectory in a GeoDataDisplay figure.

        The color of the trajectory will reflect the time taken by the trajectory.
        Optional argument colorbar_time_range may be a tuple of start and end times in seconds.
        If the Optional argument with_colorbar is set to True, a color bar will be displayed of the trajectory.
        '''
        if len(self.plan_trajectory.segments) < 2:
            return
        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        color_range = np.linspace(self.plan_trajectory.start_time() / 60, self.plan_trajectory.end_time() / 60, len(x))
        color_norm = matplotlib.colors.Normalize(vmin=color_range[0], vmax=color_range[-1])
        if colorbar_time_range is not None:
            color_norm = matplotlib.colors.Normalize(vmin=colorbar_time_range[0]/60, vmax=colorbar_time_range[1]/60)
        self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=color_range,
                                   norm=color_norm, cmap=matplotlib.cm.gist_rainbow,
                                                zorder=PlanDisplayExtension.TRAJECTORY_LAYER))
        if kwargs.get('with_colorbar', False):
            cb = self._figure.colorbar(self._drawings[-1], ax=self.axis, shrink=0.65, aspect=20)
            cb.set_label("Flight time [min]")
            self._colorbars.append(cb)

    def _draw_solid_path_extension(self, *args, **kwargs):
        '''Draw trajectory in a GeoDataDisplay figure with solid color

        kwargs:
            color: desired color. Default: C0.
        '''
        if len(self.plan_trajectory.segments) < 2:
            return
        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        color = kwargs.get('color', 'C0')
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=color,
                                                zorder=PlanDisplayExtension.TRAJECTORY_LAYER))
        # TODO: implement legend

    def _draw_segments_extension(self, *args, **kwargs):
        '''Draw observation segments with start and end points in a GeoDataDisplay figure.'''
        if len(self.plan_trajectory.segments) < 2:
            return
        color = kwargs.get('color', 'C0')
        segments = self.plan_trajectory.segments[1:-1]
        start_x = [s.start.x for s in segments]
        start_y = [s.start.y for s in segments]
        end_x = [s.end.x for s in segments]
        end_y = [s.end.y for s in segments]

        self._drawings.append(self.axis.scatter(start_x, start_y, s=10, edgecolor='black', c=color, marker='D',
                                                zorder=PlanDisplayExtension.TRAJECTORY_OVERLAY_LAYER))
        self._drawings.append(self.axis.scatter(end_x, end_y, s=10, edgecolor='black', c=color, marker='>',
                                                zorder=PlanDisplayExtension.TRAJECTORY_OVERLAY_LAYER))

        start_base = self.plan_trajectory.segments[0]
        finish_base = self.plan_trajectory.segments[-1]

        self._drawings.append(
            self.axis.scatter(start_base.start.x, start_base.start.y, s=10, edgecolor='black', c=color, marker='o',
                              zorder=PlanDisplayExtension.TRAJECTORY_OVERLAY_LAYER))
        self._drawings.append(
            self.axis.scatter(finish_base.start.x, finish_base.start.y, s=10, edgecolor='black', c=color, marker='o',
                              zorder=PlanDisplayExtension.TRAJECTORY_OVERLAY_LAYER))

        for i in range(len(segments)):
            self._drawings.append(self.axis.plot([start_x[i], end_x[i]], [start_y[i], end_y[i]], c=color, linewidth=2,
                                                 zorder=PlanDisplayExtension.TRAJECTORY_LAYER))

    def _draw_observedcells(self, observations, **kwargs):
        for ptt in observations:
            self.axis.scatter(ptt.as_tuple()[0][0], ptt.as_tuple()[0][1], s=4, c=(0., 1., 0., .5),
                              zorder=PlanDisplayExtension.RASTER_OVERLAY_LAYER, edgecolors='none', marker='s')


def plot_plan(plan, geodatadisplay, time_range: 'Optional[Tuple[float, float]]' = None, show=False):
    colors = cycle(["red", "green", "blue", "black", "magenta"])
    for traj, color in zip(plan.trajectories(), colors):
        geodatadisplay.plan_trajectory = traj
        geodatadisplay.draw_solid_path(color=color)
        geodatadisplay.draw_segments(color=color)
    if show:
        geodatadisplay.axis.get_figure().show()


def run_benchmark(scenario, save_directory, instance_name, output_options_plot: dict, output_options_planning: dict,
                  snapshots, vns_name, plot=False):
    import os
    import json
    from fire_rs.firemodel import propagation

    # Fetch scenario environment data
    env = propagation.Environment(scenario.area, wind_speed=scenario.wind_speed, wind_dir=scenario.wind_direction)

    # Propagate fires in the environment
    prop = propagation.propagate_from_points(env, scenario.ignitions, horizon=scenario.time_window_end + 60 * 10)
    ignitions = prop.ignitions()

    # Propagation was running more time than desired in order to reduce edge effect.
    # Now we need to filter out-of-range ignition times. Crop range is rounded up to the next 10-minute mark (minus 1).
    # This makes color bar ranges and fire front contour plots nicer
    ignitions['ignition'][ignitions['ignition'] > int(scenario.time_window_end / 60. + 5) * 60. - 60] = _DBL_MAX

    # If 'use_elevation' option is True, plan using a 3d environment. If not, use a flat terrain.
    # This is independent of the output plot, because it can show the real terrain even in flat terrain mode.
    terrain = env.raster.slice('elevation')
    if (output_options_planning['use_elevation'] == False):
        terrain.data['elevation'] = np.zeros_like(terrain.data['elevation'])

    # Transform altitude of UAV bases from agl (above ground level) to absolute
    for f in scenario.flights:
        base_h = terrain["elevation"][terrain.array_index(
            GeoData_Point(f.uav.base_waypoint[0], f.uav.base_waypoint[1]))]
        # The new WP is (old_x, old_y, old_z + elevation[old_x, old_y], old_dir)
        f.uav.base_waypoint = Waypoint(
            f.uav.base_waypoint[0], f.uav.base_waypoint[1], f.uav.base_waypoint[2] + base_h, f.uav.base_waypoint[3])
    # Retrieve trajectory configurations in as C++ objects
    flights = [f.as_trajectory_config() for f in scenario.flights]

    conf = {
        'min_time': scenario.time_window_start,
        'max_time': scenario.time_window_end,
        'save_every': 0,
        'save_improvements': snapshots,
        'discrete_elevation_interval': DISCRETE_ELEVATION_INTERVAL,
        'vns': vns_configurations[vns_name]
    }
    conf['vns']['configuration_name'] = vns_name

    # Call the C++ library that calculates the plan
    res = up.plan_vns(flights, ignitions.as_cpp_raster(), terrain.as_cpp_raster(), json.dumps(conf))

    plan = res.final_plan()

    # Representation of unburned cells using max double is not suitable for display,
    # so those values must be converted to NaN
    ignitions_nan = ignitions['ignition'].copy()
    ignitions_nan[ignitions_nan == _DBL_MAX] = np.nan
    first_ignition = np.nanmin(ignitions_nan)
    last_ignition = np.nanmax(ignitions_nan)

    # If 'discrete_elevation_interval' is set, discretize the elevation raster that will be displayed
    if (output_options_planning['discrete_elevation_interval']):
        a = env.raster.data['elevation'] + output_options_planning['discrete_elevation_interval']
        b = np.fmod(env.raster.data['elevation'], output_options_planning['discrete_elevation_interval'])
        env.raster.data['elevation'] = a-b

    # Create the geodatadisplay object & extensions that are going to be used
    geodatadisplay = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
    PlanDisplayExtension(None).extend(geodatadisplay)

    # Draw background layers
    for layer in output_options_plot['background']:
        if layer == 'elevation_shade':
            geodatadisplay.draw_elevation_shade(with_colorbar=output_options_plot.get('colorbar', True))
        elif layer == 'ignition_shade':
            geodatadisplay.draw_ignition_shade(with_colorbar=output_options_plot.get('colorbar', True))
        elif layer == 'observedcells':
            geodatadisplay.draw_observedcells(res.final_plan().observations())
        elif layer == 'ignition_contour':
            try:
                geodatadisplay.draw_ignition_contour(with_labels=True)
            except ValueError as e:
                logging.warn(e)
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()

    for i, t in enumerate(res.final_plan().trajectories()):
        print("traj #{} duration: {} / {}".format(i, t.duration(), t.conf.max_flight_time))

    # Plot the final plan
    plot_plan(res.final_plan(), geodatadisplay, time_range=(first_ignition, last_ignition), show=True)

    # Save the picture
    print("saving as: " + str(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png')))))
    geodatadisplay.axis.get_figure().set_size_inches(*output_options_plot.get('size', (15, 10)))
    geodatadisplay.axis.get_figure().savefig(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png'))),
        dpi=output_options_plot.get('dpi', 150), bbox_inches='tight')

    # save metadata to file
    with open(os.path.join(save_directory, instance_name+".json"), "w") as metadata_file:
        import json
        parsed = json.loads(res.metadata())
        parsed["benchmark_id"] = instance_name
        parsed["date"] = time.strftime("%Y-%m-%d--%H:%M:%S")
        print(json.dumps(parsed, indent=4), file=metadata_file)

    matplotlib.pyplot.close(geodatadisplay.axis.get_figure())


    # If intermediate plans are available, save them
    for i, i_plan in enumerate(res.intermediate_plans):
        geodatadisplay = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        PlanDisplayExtension(None).extend(geodatadisplay)
        for layer in output_options_plot['background']:
            if layer == 'elevation_shade':
                geodatadisplay.draw_elevation_shade(with_colorbar=output_options_plot.get('colorbar', True))
            elif layer == 'ignition_shade':
                geodatadisplay.draw_ignition_shade(with_colorbar=output_options_plot.get('colorbar', True))
            elif layer == 'observedcells':
                geodatadisplay.draw_observedcells(i_plan.observations())
            elif layer == 'ignition_contour':
                geodatadisplay.draw_ignition_contour(with_labels=True)
            elif layer == 'wind_quiver':
                geodatadisplay.draw_wind_quiver()
        plot_plan(i_plan, geodatadisplay, time_range=(first_ignition, last_ignition), show=True)

        i_plan_dir = os.path.join(save_directory, instance_name)
        if not os.path.exists(i_plan_dir):
            os.makedirs(i_plan_dir)

        print("saving as: " + str(
            os.path.join(i_plan_dir, str(i) + "." + str(output_options_plot.get('format', 'png')))))
        geodatadisplay.axis.get_figure().set_size_inches(*output_options_plot.get('size', (15, 10)))
        geodatadisplay.axis.get_figure().savefig(os.path.join(
            i_plan_dir, str(i) + "." + str(output_options_plot.get('format', 'png'))),
            dpi=output_options_plot.get('dpi', 150), bbox_inches='tight')
        matplotlib.pyplot.close(geodatadisplay.axis.get_figure())

    del res


def generate_scenario_newsletter():
    # 9 by 7 km area
    area = Area(545000.0, 548000.0, 6211000.0, 6214000.0)
    uav_max_pitch_angle = 6. / 180. * np.pi  # aka gamma_max=0.11rad for 2m/s rate of climb and 18m/s cruise air speed
    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymax - 100, 100., 0.)]

    num_ignitions = 1
    wind_speed = random.choice([10.,])  # 18 km/h, 36 km/h, 54 km/h, 72 km/h
    wind_dir = random.choice([np.pi/4,])
    area_range = (area.xmax - area.xmin, area.ymax - area.ymin)
    ignitions = [TimedPoint(random.uniform(area.xmin+area_range[0]*.2, area.xmax-area_range[0]*.4),
                            random.uniform(area.ymin+area_range[1]*.4, area.ymax-area_range[1]*.6),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, random.choice(uav_bases))
        uav_start = start + 3000
        max_flight_time = 450
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_singlefire_singleuav_3d():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_max_pitch_angle = 6. / 180. * np.pi  # aka gamma_max=0.11rad for 2m/s rate of climb and 18m/s cruise air speed
    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymin + 100, 100., 0.)]

    num_ignitions = 1
    wind_speed = random.choice([5., 10., 15., 20.])  # 18 km/h, 36 km/h, 54 km/h, 72 km/h
    wind_dir = random.choice([0., np.pi/2, np.pi, 3*np.pi/4])
    area_range = (area.xmax - area.xmin, area.ymax - area.ymin)
    ignitions = [TimedPoint(random.uniform(area.xmin+area_range[0]*.1, area.xmax-area_range[0]*.1),
                            random.uniform(area.ymin+area_range[1]*.1, area.ymax-area_range[1]*.1),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, random.choice(uav_bases))
        uav_start = random.uniform(start + 2500, start + 7500.)
        max_flight_time = random.uniform(1000, 1500)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_singlefire_singleuav_shortrange():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin+100, area.ymin+100, 0)]

    wind_speed = 15.
    wind_dir = 0.
    num_ignitions = 1
    x_area_range = area.xmax - area.xmin
    y_area_range = area.ymax - area.ymin
    ignitions = [TimedPoint(random.uniform(area.xmin, area.xmax-0.5*x_area_range),
                            random.uniform(area.ymin, area.ymax-0.5*y_area_range),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, random.choice(uav_bases))
        uav_start = random.uniform(start + 5000, start + 7000.)
        max_flight_time = random.uniform(500, 1000)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_singlefire_singleuav():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin +100, area.ymin+100, 0)]

    wind_speed = 15.
    wind_dir = 0.
    num_ignitions = 1
    ignitions = [TimedPoint(random.uniform(area.xmin, area.xmax),
                            random.uniform(area.ymin, area.ymax),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, random.choice(uav_bases))
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(1000, 1500)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin+100, area.ymin+100, 100, 0),
        Waypoint(area.xmin+100, area.ymax-100, 100, 0),
        Waypoint(area.xmax-100, area.ymin+100, 100, 0),
        Waypoint(area.xmax-100, area.ymax-100, 100, 0)
    ]

    wind_speed = 15.  #random.uniform(10., 20.)  # wind speed in [10,20] km/h
    wind_dir = 0.  #random.random() * 2 * np.pi
    num_ignitions = random.randint(1, 3)
    ignitions = [TimedPoint(random.uniform(area.xmin, area.xmax),
                            random.uniform(area.ymin, area.ymax),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = random.randint(1, 3)
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, random.choice(uav_bases))
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


scenario_factory_funcs = {'default': generate_scenario,
                          'singlefire_singleuav': generate_scenario_singlefire_singleuav,
                          'singlefire_singleuav_shortrange': generate_scenario_singlefire_singleuav_shortrange,
                          'singlefire_singleuav_3d': generate_scenario_singlefire_singleuav_3d,
                          'newsletter': generate_scenario_newsletter,}

max_planning_time = 5.

vns_configurations = {
    "base": {
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "dubins-opt",
                "max_trials": 200,
                "generators": [
                    {"name": "MeanOrientationChangeGenerator"},
                    {"name": "RandomOrientationChangeGenerator"},
                    {"name": "FlipOrientationChangeGenerator"}]},
            {"name": "one-insert",
                "max_trials": 50,
                "select_arbitrary_trajectory": False,
                "select_arbitrary_position": False}
        ]
    },
    "base_no_dubins": {
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "trajectory-smoothing",
                "max_trials": 200},
            {"name": "one-insert",
                "max_trials": 50,
                "select_arbitrary_trajectory": False,
                "select_arbitrary_position": False}]
    },
    "full": {
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "dubins-opt",
                "max_trials": 200,
                "generators": [
                    {"name": "RandomOrientationChangeGenerator"},
                    {"name": "FlipOrientationChangeGenerator"}]},
            {"name": "one-insert",
                "max_trials": 50,
                "select_arbitrary_trajectory": False,
                "select_arbitrary_position": False},
            {"name": "one-insert",
                "max_trials": 200,
                "select_arbitrary_trajectory": True,
                "select_arbitrary_position": False},
            {"name": "one-insert",
                "max_trials": 200,
                "select_arbitrary_trajectory": True,
                "select_arbitrary_position": True}
        ]
    },
    "insert_traj": {
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "dubins-opt",
                "max_trials": 200,
                "generators": [
                    {"name": "RandomOrientationChangeGenerator"},
                    {"name": "FlipOrientationChangeGenerator"}]},
            {"name": "one-insert",
                "max_trials": 300,
                "select_arbitrary_trajectory": True,
                "select_arbitrary_position": False},
        ]
    },
    "insert_pos": {
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "dubins-opt",
                "max_trials": 200,
                "generators": [
                    {"name": "RandomOrientationChangeGenerator"},
                    {"name": "FlipOrientationChangeGenerator"}]},
            {"name": "one-insert",
                "max_trials": 300,
                "select_arbitrary_trajectory": True,
                "select_arbitrary_position": True},
        ]
    }
}


def main():
    import pickle
    import os
    import argparse
    import joblib
    import json

    from fire_rs.geodata.environment import DEFAULT_FIRERS_DATA_FOLDER


    class JsonReadAction(argparse.Action):

        def __call__(self, parser, namespace, values, option_string=None):
            try:
               return json.load(values)
            except json.JSONDecodeError as err:
                raise argparse.ArgumentError(self, err)


    # CLI argument parsing
    FROMFILE_PREFIX_CHARS = '@'
    parser = argparse.ArgumentParser(
        prog='benchmark.py', fromfile_prefix_chars=FROMFILE_PREFIX_CHARS,
        epilog="The arguments that start with `" + FROMFILE_PREFIX_CHARS + \
               "' will be treated as files, and will be replaced by the arguments they contain.")
    parser.add_argument("--name",
                        help="name of the benchmark. The resulting folder name will be prefixed by 'benchmark_'.",
                        choices=scenario_factory_funcs.keys())
    parser.add_argument("--folder",
                        help="Name of the folder in which benchmarks are to be saved.",
                        default=DEFAULT_FIRERS_DATA_FOLDER)
    parser.add_argument("--background", nargs='+',
                        help="List of background layers for the output figures, from bottom to top.",
                        choices=['elevation_shade', 'ignition_shade', 'observedcells', 'ignition_contour', 'wind_quiver'],
                        default=['elevation_shade', 'ignition_contour', 'wind_quiver'])
    parser.add_argument('--colorbar', dest='colorbar', action='store_true',
                        help="Display colorbars")
    parser.add_argument('--no-colorbar', dest='colorbar', action='store_false',
                        help="Display colorbars")
    parser.set_defaults(colorbar=True)
    parser.add_argument("--format",
                        help="Format of the output figures",
                        choices=['png', 'svg', 'eps', 'pdf'],
                        default='png')
    parser.add_argument("--vns-conf", action=JsonReadAction, type=argparse.FileType('r'),
                        help="Load VNS configurations from a JSON file")
    parser.add_argument("--vns",
                        help="Select a VNS configuration, among the default ones or from the file specified in --vns-conf.",
                        default='base')
    parser.add_argument("--elevation",
                        help="Source of elevation considered by the planning algorithm",
                        choices=['flat', 'dem', 'discrete'],
                        default='dem')
    parser.add_argument("--num_instances",
                        help="Number of benchmarks to generate if needed",
                        default=40)
    parser.add_argument("--dpi",
                        help="Resolution of the output figures",
                        type=int,
                        default=150)
    parser.add_argument("--size", nargs=2,
                        help="Size (in inches) of the output figures",
                        type=int,
                        default=(15,10))
    parser.add_argument("--instance",
                        help="Runs a particular benchmark instance",
                        type=int,
                        default=None)
    parser.add_argument("--snapshots", action="store_true",
                        help="Save snapshots of the plan after every improvement. Beware, this option will slowdown the simulation and consume lots of memory",
                        default=False)
    parser.add_argument("--parallel",
                        help="Set the number of threads to be used for parallel processing.",
                        type=int,
                        default=1)
    parser.add_argument("--wait", action="store_true",
                        help="Wait for user input before start. Useful to hold the execution while attaching to a debugger")
    args = parser.parse_args()

    # Use TeX fonts when the output format is eps or pdf
    # When this is active, matplotlib drawing is slower
    if args.format == 'eps' or args.format == 'pdf':
        matplotlib.rcParams['font.family'] = 'serif'
        matplotlib.rcParams['text.usetex'] = True

    # TODO: the vns conf file is not picked up (definitions used are the one defined in this file)
    # if args.vns_conf:
    #     vns_configurations = vars(args.vns_conf)

    # Set-up output options
    output_options = {'plot':{}, 'planning':{}, }
    output_options['plot']['background'] = args.background
    output_options['plot']['format'] = args.format
    output_options['plot']['colorbar'] = args.colorbar
    output_options['plot']['dpi'] = args.dpi
    output_options['plot']['size'] = args.size
    output_options['planning']['use_elevation'] = False if args.elevation == "flat" else True
    output_options['planning']['discrete_elevation_interval'] = \
        DISCRETE_ELEVATION_INTERVAL if args.elevation == "discrete" else 0

    # benchmark folder handling
    benchmark_name = args.name
    benchmark_name_full = "benchmark"
    if benchmark_name:
        benchmark_name_full = "_".join([benchmark_name_full, str(benchmark_name)])
    benchmark_dir = os.path.join(args.folder, benchmark_name_full)
    if not os.path.exists(benchmark_dir):
        os.makedirs(benchmark_dir)

    # Scenario loading / generation
    scenarios_file = os.path.join(benchmark_dir, "scenarios.dump")
    if not os.path.exists(scenarios_file):
        scenario_generator = scenario_factory_funcs['default']
        if benchmark_name in scenario_factory_funcs:
            scenario_generator = scenario_factory_funcs[benchmark_name]
        scenarios = [scenario_generator() for i in range(int(args.num_instances))]
        pickle.dump(scenarios, open(scenarios_file, "wb"))
    else:
        scenarios = pickle.load(open(scenarios_file, "rb"))

    if benchmark_name:
        logging.info("Running benchmark %s from '%s'", benchmark_name, benchmark_dir)
    else:
        logging.info("Running default benchmark from '%s'", benchmark_dir)

    # Current date and time string
    run_id = time.strftime("%Y-%m-%d--%H:%M:%S")

    # Current run folder
    run_dir = os.path.join(benchmark_dir, run_id)
    if not os.path.exists(run_dir):
        os.makedirs(run_dir)

    if args.wait:
        input("Press enter to continue...")

    if args.instance != None:
        to_run = [(args.instance, scenarios[args.instance])]
    else:
        to_run = enumerate(scenarios)

    joblib.Parallel(n_jobs=args.parallel, backend="threading", verbose=5)\
        (joblib.delayed(run_benchmark)(s, run_dir, str(i), output_options_plot=output_options['plot'],
                                       snapshots=args.snapshots, output_options_planning=output_options['planning'],
                                       vns_name=args.vns) for i, s in to_run)


if __name__=='__main__':
    main()
