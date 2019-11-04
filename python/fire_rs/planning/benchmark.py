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

import datetime
import pickle
import os
import argparse
import joblib
import json
import sys
import json
import logging
import numpy as np
import os
import random
import typing as ty
import pytz

from collections import namedtuple
from collections.abc import Sequence
from typing import Optional, Tuple, Union

import matplotlib

matplotlib.use('Agg')  # do not require X display to plot figures that are not shown
import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot

import fire_rs.uav_planning as up
import fire_rs.firemodel.propagation
import fire_rs.geodata.environment
import fire_rs.geodata.geo_data

from fire_rs.firemodel import propagation
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.geodata.geo_data import TimedPoint, Area, GeoData
from fire_rs.planning.display import plot_plan_with_background, TrajectoryDisplayExtension
from fire_rs.planning.new_planning import TrajectoryConfig,\
    UAV, VNSConfDB, Waypoint, FireData, make_fire_data, WindVector, TimeWindow,\
    make_utility_map, Plan, Planner, UAVModels

_DBL_MAX = np.inf
DISCRETE_ELEVATION_INTERVAL = 100

_logger = logging.getLogger(__name__)
_logger.setLevel(logging.DEBUG)


class FlightConf:

    def __init__(self, name: str, uav: UAV, start_time: float, max_flight_time: float,
                 base_waypoint: 'Waypoint',
                 finalbase_waypoint: 'Optional[Waypoint]' = None,
                 wind: 'Tuple[float, float]' = (0., 0.)):
        assert start_time > 0
        self.uav = uav  # type: UAV
        self.base_waypoint = base_waypoint  # type: Waypoint
        # type: Optional[Waypoint]
        self.finalbase_waypoint = finalbase_waypoint if finalbase_waypoint else base_waypoint
        self.start_time = start_time  # type: float
        self.wind = wind
        self.name = name
        self.max_flight_time = max_flight_time

    def as_cpp(self):
        return TrajectoryConfig(self.uav, self.base_waypoint,
                                self.finalbase_waypoint, self.start_time,
                                self.max_flight_time,
                                WindVector(self.wind[0], self.wind[1]))

    def __repr__(self):
        return "".join(("FlightConf(",
                        "name=", repr(self.name), ",",
                        "uav=", repr(self.uav), ",",
                        "start_time=", repr(self.start_time), ",",
                        "max_flight_time=", repr(self.max_flight_time), ",",
                        "base_waypoint=", repr(self.base_waypoint), ",",
                        "finalbase_waypoint=", repr(self.finalbase_waypoint), ",",
                        "wind=", repr(self.wind), ")"))


class PlanningEnvironment(fire_rs.firemodel.propagation.Environment):
    """Propagation environment with optional discrete elevation only for planning """

    def __init__(self, area, wind_speed, wind_dir, planning_elevation_mode: 'str' = 'dem',
                 discrete_elevation_interval: 'int' = 0, flat_altitude=0., world=None):
        super().__init__(area, wind_speed, wind_dir, world=world)

        self.planning_elevation_mode = planning_elevation_mode
        self.discrete_elevation_interval = discrete_elevation_interval

        self._flat_altitude = flat_altitude

        elev_planning = None

        if self.planning_elevation_mode == 'flat':
            elev_planning = self.raster.clone(fill_value=flat_altitude,
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'dem':
            elev_planning = self.raster.clone(data_array=self.raster["elevation"],
                                              dtype=[('elevation_planning', 'float64')])
        elif self.planning_elevation_mode == 'discrete':
            a = self.raster.data['elevation'] + self.discrete_elevation_interval
            b = np.fmod(self.raster.data['elevation'], self.discrete_elevation_interval)
            elev_planning = self.raster.clone(data_array=a - b,
                                              dtype=[('elevation_planning', 'float64')])
        else:
            elev_planning = self.raster.clone(data_array=self.raster["elevation"],
                                              dtype=[('elevation_planning', 'float64')])
            raise ValueError("".join(["Wrong planning_elevation_mode '",
                                      self.planning_elevation_mode, "'"]))

        self.raster = self.raster.combine(elev_planning)  # type: fire_rs.geodata.geo_data.GeoData


class Scenario:
    """Benchmark scenario"""

    def __init__(self, area: ty.Tuple[ty.Tuple[float, float], ty.Tuple[float, float]],
                 wind_speed: float,
                 wind_direction: float,
                 ignitions: ty.Sequence[TimedPoint],
                 flights: ty.Sequence[FlightConf]):
        self.area = area  # type: Area
        self.wind_speed = wind_speed  # type: float
        self.wind_direction = wind_direction  # type: float
        assert len(ignitions) > 0
        self.ignitions = ignitions  # type: ty.Sequence[TimedPoint]
        assert len(flights) > 0
        self.flights = flights  # type: ty.Sequence[FlightConf]

        self.time_window_start = np.inf  # type: float
        self.time_window_end = -np.inf  # type: float
        for flight in flights:
            self.time_window_start = min(self.time_window_start, flight.start_time - 180)
            self.time_window_end = max(self.time_window_end,
                                       flight.start_time + flight.max_flight_time + 180)

    def __repr__(self):
        return "".join(["Scenario(area=", repr(self.area), ", wind_speed=", repr(self.wind_speed),
                        ", wind_direction=", repr(self.wind_direction), ", ignitions=",
                        repr(self.ignitions),
                        ", traj_conf=", repr(self.flights), ")"])


def run_benchmark(scenario: Scenario, save_directory: str, instance_name: str,
                  output_options_plot: ty.Mapping[str, ty.Any],
                  output_options_planning: ty.Mapping[str, ty.Any],
                  snapshots: bool, vns_name: str, output_options_data: ty.Mapping[str, ty.Any]):
    _logger.info("Starting benchmark instance %s", instance_name)

    # Fetch scenario environment data with additional elevation mode for planning
    env = PlanningEnvironment(scenario.area, wind_speed=scenario.wind_speed,
                              wind_dir=scenario.wind_direction,
                              planning_elevation_mode=output_options_planning['elevation_mode'],
                              discrete_elevation_interval=DISCRETE_ELEVATION_INTERVAL,
                              world=fire_rs.geodata.environment.World(
                                  landcover_to_fuel_remap=fire_rs.geodata.environment.EVERYTHING_FUELMODEL_REMAP))

    # Propagate fires in the environment
    propagation_end_time = scenario.time_window_end + 60 * 10
    _logger.debug("Propagating fire ignitions %s until %s", str(scenario.ignitions),
                  str(propagation_end_time))
    prop = propagation.propagate_from_points(env, scenario.ignitions, until=propagation_end_time)
    ignitions = prop.ignitions()

    # Transform altitude of UAV bases from agl (above ground level) to absolute
    for f in scenario.flights:
        base_h = 0.  # If flat, start at the default segment insertion h
        if output_options_planning['elevation_mode'] != 'flat':
            base_h = base_h + env.raster["elevation"][env.raster.array_index((f.base_waypoint.x,
                                                                              f.base_waypoint.y))]
        # The new WP is (old_x, old_y, old_z + elevation[old_x, old_y], old_dir)
        f.base_waypoint = Waypoint(f.base_waypoint.x, f.base_waypoint.y, base_h,
                                   f.base_waypoint.dir)

    conf = {
        'min_time': scenario.time_window_start,
        'max_time': scenario.time_window_end,
        'save_every': 0,
        'save_improvements': snapshots,
        'vns': vns_configurations[vns_name]
    }
    conf['vns']['configuration_name'] = vns_name

    # Define the flight window
    flight_window = TimeWindow(scenario.time_window_start, scenario.time_window_end)

    # Create utility map
    utility = make_utility_map(ignitions, flight_window, layer='ignition', output_layer='utility')
    utility_cpp = utility.as_cpp_raster('utility')

    # Make a FireData object
    fire_data = make_fire_data(ignitions, env.raster, elevation_map_layer='elevation_planning')

    # Create an initial plan
    initial_plan = Plan(instance_name, [f.as_cpp() for f in scenario.flights], fire_data,
                        flight_window, utility_cpp)

    # Set up a Planner object from the initial Plan
    pl = Planner(initial_plan, vns_configurations[vns_name])
    pl.save_improvements = snapshots
    pl.save_every = 0

    search_result = pl.compute_plan()
    final_plan = search_result.final_plan()

    # Propagation was running more time than desired in order to reduce edge effect.
    # Now we need to filter out-of-range ignition times. Crop range is rounded up to the next
    # 10-minute mark (minus 1). This makes color bar ranges and fire front contour plots nicer
    ignitions['ignition'][ignitions['ignition'] > \
                          int(scenario.time_window_end / 60. + 5) * 60. - 60] = _DBL_MAX
    # Representation of unburned cells using max double is not suitable for display,
    # so those values must be converted to NaN
    ignitions_nan = ignitions['ignition'].copy()
    ignitions_nan[ignitions_nan == _DBL_MAX] = np.nan
    first_ignition = np.nanmin(ignitions_nan)
    last_ignition = np.nanmax(ignitions_nan)

    # Create the geodatadisplay object & extensions that are going to be used
    geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions), frame=(0, 0))
    geodatadisplay.add_extension(TrajectoryDisplayExtension, (None,), {})

    plot_plan_with_background(final_plan, geodatadisplay, (first_ignition, last_ignition),
                              output_options_plot)

    # Save the picture
    filepath = os.path.join(save_directory,
                            instance_name + "." + str(output_options_plot.get('format', 'png')))
    _logger.info("Saving as figure as: %s", str(filepath))
    geodatadisplay.axes.get_figure().set_size_inches(*output_options_plot.get('size', (15, 10)))
    geodatadisplay.axes.get_figure().savefig(filepath, dpi=output_options_plot.get('dpi', 150),
                                             bbox_inches='tight')

    # Save rasters
    if output_options_data['save_rasters']:
        raster_data_dir = os.path.join(save_directory, instance_name + "_data")
        if not os.path.exists(raster_data_dir):
            os.makedirs(raster_data_dir)
        env.raster.write_to_file(os.path.join(raster_data_dir, ".".join(
            ("_".join((instance_name, "elevation_planning")), 'tif'))), 'elevation_planning')
        env.raster.write_to_file(os.path.join(raster_data_dir, ".".join(
            ("_".join((instance_name, "wind_velocity")), 'tif'))), 'wind_velocity')
        env.raster.write_to_file(os.path.join(raster_data_dir, ".".join(
            ("_".join((instance_name, "wind_angle")), 'tif'))), 'wind_angle')
        prop.ignitions().write_to_file(
            os.path.join(raster_data_dir, ".".join(("_".join((instance_name, "ignition")), 'tif'))),
            'ignition', nodata=np.inf)
        u_initial_map = GeoData.from_cpp_raster(initial_plan.utility_map(), "utility",
                                                projection=utility.projection)
        u_initial_map.write_to_file(os.path.join(raster_data_dir, ".".join(
            ("_".join((instance_name, "utility_initial")), 'tif'))), 'utility')
        u_final_map = GeoData.from_cpp_raster(final_plan.utility_map(), "utility",
                                              projection=utility.projection)
        u_final_map.write_to_file(os.path.join(raster_data_dir, ".".join(
            ("_".join((instance_name, "utility_final")), 'tif'))), 'utility')

    # print([o.as_tuple() for o in final_plan.observations()])

    # Log planned trajectories metadata
    for i, t in enumerate(final_plan.trajectories()):
        _logger.debug("traj #{} duration: {} / {}".format(i, t.duration(), t.conf.max_flight_time))

    # save metadata to file
    with open(os.path.join(save_directory, instance_name + ".json"), "w") as metadata_file:
        import json
        parsed = json.loads(search_result.metadata())
        parsed["benchmark_id"] = instance_name
        parsed["date"] = datetime.datetime.now().isoformat()
        metadata_file.write(json.dumps(parsed, indent=4))

    matplotlib.pyplot.close(geodatadisplay.axes.get_figure())

    # If intermediate plans are available, save them
    for i, plan in enumerate(search_result.intermediate_plans):
        geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        geodatadisplay.add_extension(TrajectoryDisplayExtension, (None,), {})
        plot_plan_with_background(plan, geodatadisplay, (first_ignition, last_ignition),
                                  output_options_plot)

        i_plan_dir = os.path.join(save_directory, instance_name)
        if not os.path.exists(i_plan_dir):
            os.makedirs(i_plan_dir)

        filepath = os.path.join(i_plan_dir,
                                str(i) + "." + str(output_options_plot.get('format', 'png')))

        _logger.info("Saving intermediate plan as: %s", str(filepath))
        geodatadisplay.axes.get_figure().set_size_inches(*output_options_plot.get('size', (15, 10)))
        geodatadisplay.axes.get_figure().savefig(filepath, dpi=output_options_plot.get('dpi', 150),
                                                 bbox_inches='tight')
        matplotlib.pyplot.close(geodatadisplay.axes.get_figure())

    del search_result


def generate_scenario_newsletter():
    # 9 by 7 km area
    area = Area(545000.0, 548000.0, 6211000.0, 6214000.0)
    uav_max_pitch_angle = 6. / 180. * np.pi  # aka gamma_max=0.11rad for 2m/s rate of climb and 18m/s cruise air speed
    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymax - 100, 100., 0.)]

    num_ignitions = 1
    wind_speed = random.choice([10., ])  # 18 km/h, 36 km/h, 54 km/h, 72 km/h
    wind_dir = random.choice([np.pi / 4, ])
    area_range = (area.xmax - area.xmin, area.ymax - area.ymin)
    ignitions = [
        TimedPoint(random.uniform(area.xmin + area_range[0] * .2, area.xmax - area_range[0] * .4),
                   random.uniform(area.ymin + area_range[1] * .4, area.ymax - area_range[1] * .6),
                   random.uniform(0, 3000))
        for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav_start = start + 3000
        max_flight_time = 450
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_utility_test():
    # 2 by 2 km area
    area = Area(480060.0, 482060.0, 6210074.0, 6212074.0)
    uav_max_pitch_angle = 6. / 180. * np.pi  # aka gamma_max=0.11rad for 2m/s rate of climb and 18m/s cruise air speed
    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymin + 100, 0., 0.)]

    num_ignitions = 1
    wind_speed = random.choice([5., 10., 15., 20.])  # 18 km/h, 36 km/h, 54 km/h, 72 km/h
    wind_dir = random.choice([0., np.pi / 2, np.pi, 3 * np.pi / 4])
    area_range = (area.xmax - area.xmin, area.ymax - area.ymin)
    ignitions = [
        TimedPoint(random.uniform(area.xmin + area_range[0] * .1, area.xmax - area_range[0] * .1),
                   random.uniform(area.ymin + area_range[1] * .1, area.ymax - area_range[1] * .1),
                   random.uniform(0, 3000))
        for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 2500, start + 7500.)
        max_flight_time = random.uniform(1000, 1500)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

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
    wind_dir = random.choice([0., np.pi / 2, np.pi, 3 * np.pi / 4])
    area_range = (area.xmax - area.xmin, area.ymax - area.ymin)
    ignitions = [
        TimedPoint(random.uniform(area.xmin + area_range[0] * .1, area.xmax - area_range[0] * .1),
                   random.uniform(area.ymin + area_range[1] * .1, area.ymax - area_range[1] * .1),
                   random.uniform(0, 3000))
        for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 2500, start + 7500.)
        max_flight_time = random.uniform(1000, 1500)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_singlefire_singleuav_shortrange():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymin + 100, 0)]

    wind_speed = 15.
    wind_dir = 0.
    num_ignitions = 1
    x_area_range = area.xmax - area.xmin
    y_area_range = area.ymax - area.ymin
    ignitions = [TimedPoint(random.uniform(area.xmin, area.xmax - 0.5 * x_area_range),
                            random.uniform(area.ymin, area.ymax - 0.5 * y_area_range),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = 1
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 5000, start + 7000.)
        max_flight_time = random.uniform(500, 1000)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_singlefire_singleuav():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180 / 2  # Consider a more conservative turn rate
    uav_bases = [Waypoint(area.xmin + 100, area.ymin + 100, 0, 0)]

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
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(1000, 1500)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario

def generate_scenario_lambert93():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = 15.  # random.uniform(10., 20.)  # wind speed in [10,20] km/h
    wind_dir = 0.  # random.random() * 2 * np.pi
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
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_few():
    """A dozen UAVs monitoring a dozen fires"""
    # 9 by 7 km area
    area = Area(532000.0, 540000.0, 4567000.0, 4575000.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = random.uniform(5., 10.)  # wind speed in [10,20] km/h
    wind_dir = random.random() * 2 * np.pi
    num_ignitions = random.randint(2, 4)

    # Calculate a safe area for the ignitions
    ignitions = [TimedPoint(random.uniform(area.xmin + 100, area.xmax - 100),
                            random.uniform(area.ymin + 100, area.ymax - 100),
                            random.uniform(0, 3600))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = random.randint(2,4)
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 7100., start + 7300.)
        max_flight_time = random.uniform(1200, 2400)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_dozens():
    """A dozen UAVs monitoring a dozen fires"""
    # 9 by 7 km area
    area = Area(532000.0, 540000.0, 4567000.0, 4575000.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = random.uniform(5., 10.)  # wind speed in [10,20] km/h
    wind_dir = random.random() * 2 * np.pi
    num_ignitions = random.randint(10, 14)

    # Calculate a safe area for the ignitions
    ignitions = [TimedPoint(random.uniform(area.xmin + 100, area.xmax - 100),
                            random.uniform(area.ymin + 100, area.ymax - 100),
                            random.uniform(0, 6000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = random.randint(10, 14)
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 3999., start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario_big_fire():
    """A dozen UAVs monitoring a dozen fires"""
    # 9 by 7 km area
    area = Area(532000.0, 540000.0, 4567000.0, 4575000.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = random.uniform(1., 2.)  # wind speed in [10,20] km/h
    wind_dir = random.random() * 2 * np.pi
    num_ignitions = random.randint(1, 1)

    # Calculate a safe area for the ignitions
    allowed_range_x = (area.xmax - area.xmin) * 0.25
    allowed_range_y = (area.ymax - area.ymin) * 0.25
    ignitions = [
        TimedPoint(random.uniform(area.xmin + allowed_range_x, area.xmax - allowed_range_x),
                   random.uniform(area.ymin + allowed_range_y, area.ymax - allowed_range_y),
                   random.uniform(0, 0))
        for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = random.randint(1, 5)
    flights = []
    for i in range(num_flights):
        uav_start = random.uniform(start + 50000., start + 50000.)
        max_flight_time = random.uniform(500, 1200)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def generate_scenario():
    # 9 by 7 km area
    area = Area(532000.0, 540000.0, 4567000.0, 4575000.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = 15.  # random.uniform(10., 20.)  # wind speed in [10,20] km/h
    wind_dir = 0.  # random.random() * 2 * np.pi
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
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


scenario_factory_funcs = {'default': generate_scenario,
                          'few': generate_scenario_few,
                          'dozen': generate_scenario_dozens,
                          'big_fire': generate_scenario_big_fire,
                          'default_lambert93': generate_scenario_lambert93,
                          'utility_test': generate_scenario_utility_test,
                          'singlefire_singleuav': generate_scenario_singlefire_singleuav,
                          'singlefire_singleuav_shortrange': generate_scenario_singlefire_singleuav_shortrange,
                          'singlefire_singleuav_3d': generate_scenario_singlefire_singleuav_3d,
                          'newsletter': generate_scenario_newsletter
                          }

vns_configurations = VNSConfDB.demo_db()


def run_command(args):
    # Use TeX fonts when the output format is eps or pdf
    # When this is active, matplotlib drawing is slower
    if args.format == 'eps' or args.format == 'pdf':
        matplotlib.rcParams['font.family'] = 'serif'
        matplotlib.rcParams['text.usetex'] = True

    if args.vns_conf:
        global vns_configurations
        vns_configurations = VNSConfDB(args.vns_conf)

    # Set-up output options
    output_options = {'plot': {}, 'planning': {}, 'data': {}}
    output_options['plot']['background'] = args.background
    output_options['plot']['foreground'] = args.foreground
    output_options['plot']['format'] = args.format
    output_options['plot']['colorbar'] = args.colorbar
    output_options['plot']['dpi'] = args.dpi
    output_options['plot']['size'] = args.size
    output_options['planning']['elevation_mode'] = args.elevation
    output_options['planning']['discrete_elevation_interval'] = \
        DISCRETE_ELEVATION_INTERVAL if args.elevation == 'discrete' else 0
    output_options['data']['save_rasters'] = args.save_rasters

    # Scenario loading / generation
    scenarios = pickle.load(open(os.path.join(args.scenario, 'scenario'), 'rb'))

    # Current date and time string
    run_id = datetime.datetime.now(pytz.timezone('UTC')).isoformat()

    # Current run folder
    run_dir = os.path.join(args.scenario, run_id)
    if not os.path.exists(run_dir):
        os.makedirs(run_dir)

        # Setup logging for this benchmark run
        log_formatter = logging.Formatter(
            '%(asctime)-23s %(levelname)s [0x%(thread)x:%(name)s]: %(message)s')

        # Set configuration for the default logger (inherited by others)
        root_logger = logging.getLogger()
        # Print INFO level messages
        log_stdout_hldr = logging.StreamHandler(sys.stdout)
        log_stdout_hldr.setLevel(logging.INFO)
        log_stdout_hldr.setFormatter(log_formatter)
        root_logger.addHandler(log_stdout_hldr)
        # Log everithing to file
        log_file_hldr = logging.FileHandler(os.path.join(run_dir, ".".join((run_id, "log"))))
        log_file_hldr.setFormatter(log_formatter)
        log_file_hldr.setLevel(logging.NOTSET)
        root_logger.addHandler(log_file_hldr)

        # Use a child logger in SAOP c++ library
        up.set_logger(_logger.getChild("uav_planning"))

    _logger.info("Using benchmark scenario %s", args.scenario)
    _logger.info("Saving results in %s", run_dir)

    if args.instance is not None:
        to_run = [(args.instance, scenarios[args.instance])]
    else:
        to_run = enumerate(scenarios)

    joblib.Parallel(n_jobs=args.parallel, backend="threading", verbose=5)\
        (joblib.delayed(run_benchmark)(s, run_dir, str(i),
                                       output_options_plot=output_options['plot'],
                                       snapshots=args.snapshots,
                                       output_options_planning=output_options['planning'],
                                       vns_name=args.vns,
                                       output_options_data=output_options['data'])\
         for i, s in to_run)


def create_command(args):
    # Scenario loading / generation
    if not os.path.exists(args.output):
        os.makedirs(args.output)
    _logger.info("Using factory %s to create %d scenario instances in %s", args.factory,
                 args.n_instances, args.output)
    scenario_generator = scenario_factory_funcs[args.factory]
    scenarios = [scenario_generator() for i in range(args.n_instances)]
    with open(os.path.join(args.output, 'scenario'), 'wb+') as scenario_f:
        pickle.dump(scenarios, scenario_f)


def main():
    class JsonReadAction(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            try:
                vnsconf = json.load(values)
                setattr(namespace, self.dest, vnsconf)
            except json.JSONDecodeError as err:
                raise argparse.ArgumentError(self, err)

    # CLI argument parsing
    FROMFILE_PREFIX_CHARS = '@'
    parser = argparse.ArgumentParser(
        prog='benchmark.py', fromfile_prefix_chars=FROMFILE_PREFIX_CHARS,
        epilog="The arguments that start with `" + FROMFILE_PREFIX_CHARS + \
               "' will be treated as files, and will be replaced by the arguments they contain.")
    parser.add_argument("--wait", action="store_true",
                        help="Wait for user input before start. Useful to hold the execution while attaching to a debugger")

    subparsers = parser.add_subparsers(dest='sub_command', help='Available sub-commands')

    # create the parser for the "create" command
    parser_create = subparsers.add_parser('create', help='Create a benchmark scenario')
    parser_create.add_argument('factory',
                               help="Name of the benchmark scenario factory.",
                               choices=scenario_factory_funcs.keys())
    parser_create.add_argument('output', help="Scenario output directory", )
    parser_create.add_argument("--n-instances",
                               help="Number of scenario instances to generate",
                               type=int,
                               default=40)

    # create the parser for the "run" command
    parser_run = subparsers.add_parser('run', help='Run a benchmark scenario')

    parser_run.add_argument("scenario",
                            help="Scenario directory")
    parser_run.add_argument("--background", nargs='+',
                            help="List of background layers for the output figures, from bottom to top.",
                            choices=['elevation_shade', 'elevation_planning_shade',
                                     'ignition_shade',
                                     'observedcells', 'ignition_contour', 'wind_quiver',
                                     'utilitymap'],
                            default=['elevation_shade', 'ignition_contour', 'wind_quiver'])
    parser_run.add_argument("--foreground", nargs='+',
                            help="List of plan information foreground layers for the output figures",
                            choices=['trajectory_solid', 'arrows', 'bases'],
                            default=['trajectory_solid', 'arrows', 'bases'])
    parser_run.add_argument('--colorbar', dest='colorbar', action='store_true',
                            help="Display colorbars")
    parser_run.add_argument('--no-colorbar', dest='colorbar', action='store_false',
                            help="Display colorbars")
    parser_run.set_defaults(colorbar=True)
    parser_run.add_argument("--format",
                            help="Format of the output figures",
                            choices=['png', 'svg', 'eps', 'pdf'],
                            default='png')
    parser_run.add_argument("--vns-conf", action=JsonReadAction, type=argparse.FileType('r'),
                            help="Load VNS configurations from a JSON file. The full path must be used.")
    parser_run.add_argument("--vns",
                            help="Select a VNS configuration, among the default ones or from the file specified in --vns-conf.",
                            default='demo')
    parser_run.add_argument("--elevation",
                            help="Source of elevation considered by the planning algorithm",
                            choices=['flat', 'dem', 'discrete'],
                            default='dem')
    parser_run.add_argument("--dpi",
                            help="Resolution of the output figures",
                            type=int,
                            default=150)
    parser_run.add_argument("--size", nargs=2,
                            help="Size (in inches) of the output figures",
                            type=int,
                            default=(15, 10))
    parser_run.add_argument("--instance",
                            help="Runs a particular benchmark instance",
                            type=int,
                            default=None)
    parser_run.add_argument("--snapshots", action="store_true",
                            help="Save snapshots of the plan after every improvement. Beware, this option will slowdown the simulation and consume lots of memory",
                            default=False)
    parser_run.add_argument("--parallel",
                            help="Set the number of threads to be used for parallel processing.",
                            type=int,
                            default=1)
    parser_run.add_argument('--save-rasters', dest='save_rasters', action='store_true',
                            help="Save all the rasters generated during the benchmark.")
    parser_run.add_argument('--no-save-rasters', dest='save_rasters', action='store_false',
                            help="Do not save any of the rasters generated during the benchmark.")
    args = parser.parse_args()

    if args.wait:
        input("Press enter to continue...")

    # Log command line arguments
    _logger.info(' '.join(sys.argv))

    if args.sub_command == 'create':
        create_command(args)
    elif args.sub_command == 'run':
        run_command(args)
    else:
        parser


if __name__ == '__main__':
    main()
