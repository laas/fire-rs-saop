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

import json
import logging
import numpy as np
import os
import random
import time
import types

from collections import namedtuple, Sequence
from typing import Optional, Tuple, Union

import matplotlib
matplotlib.use('Agg')  # do not require X display to plot figures that are not shown
import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot

import fire_rs.uav_planning as up

from fire_rs.firemodel import propagation
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.geodata.geo_data import TimedPoint, Area
from fire_rs.geodata.geo_data import Point as GeoData_Point
from fire_rs.planning.display import plot_plan_with_background, TrajectoryDisplayExtension
from fire_rs.planning.planning import FlightConf, Planner, PlanningEnvironment, UAVConf, Waypoint


_DBL_MAX = np.finfo(np.float64).max
DISCRETE_ELEVATION_INTERVAL = 100


class Scenario:

    def __init__(self, area: ((float, float), (float, float)),
                 wind_speed: float,
                 wind_direction: float,
                 ignitions: [TimedPoint],
                 flights: [FlightConf]):
        self.area = area  # type: Area
        self.wind_speed = wind_speed  # type: float
        self.wind_direction = wind_direction  # type: float
        assert len(ignitions) > 0
        self.ignitions = ignitions  # type: [TimedPoint]
        assert len(flights) > 0
        self.flights = flights  # type: [FlightConf]

        self.time_window_start = np.inf
        self.time_window_end = -np.inf
        for flight in flights:
            self.time_window_start = min(self.time_window_start, flight.start_time - 180)
            self.time_window_end = max(self.time_window_end, flight.start_time+flight.max_flight_time + 180)

    def __repr__(self):
        return "".join(["Scenario(area=", repr(self.area), ", wind_speed=", repr(self.wind_speed),
                        ", wind_direction=", repr(self.wind_direction), ", ignitions=", repr(self.ignitions),
                        ", flights=", repr(self.flights), ")"])


def run_benchmark(scenario, save_directory, instance_name, output_options_plot: dict, output_options_planning: dict,
                  snapshots, vns_name, plot=False):

    # Fetch scenario environment data with additional elevation mode for planning
    env = PlanningEnvironment(scenario.area, wind_speed=scenario.wind_speed,
                              wind_dir=scenario.wind_direction,
                              planning_elevation_mode=output_options_planning['elevation_mode'],
                              discrete_elevation_interval=DISCRETE_ELEVATION_INTERVAL)

    # Propagate fires in the environment
    prop = propagation.propagate_from_points(env, scenario.ignitions,
                                             horizon=scenario.time_window_end + 60 * 10)
    ignitions = prop.ignitions()

    # Propagation was running more time than desired in order to reduce edge effect.
    # Now we need to filter out-of-range ignition times. Crop range is rounded up to the next
    # 10-minute mark (minus 1). This makes color bar ranges and fire front contour plots nicer
    ignitions['ignition'][ignitions['ignition'] > \
                          int(scenario.time_window_end / 60. + 5) * 60. - 60] = _DBL_MAX

    # Transform altitude of UAV bases from agl (above ground level) to absolute
    for f in scenario.flights:
        base_h = 100  # If flat, start at the default segment insertion h
        if output_options_planning['elevation_mode'] != 'flat':
            base_h = env.raster["elevation"]\
                [env.raster.array_index((f.base_waypoint[0], f.base_waypoint[1]))]
        # The new WP is (old_x, old_y, old_z + elevation[old_x, old_y], old_dir)
        f.base_waypoint = Waypoint(f.base_waypoint[0], f.base_waypoint[1],
                                   f.base_waypoint[2] + base_h, f.base_waypoint[3])

    conf = {
        'min_time': scenario.time_window_start,
        'max_time': scenario.time_window_end,
        'save_every': 0,
        'save_improvements': snapshots,
        'vns': vns_configurations[vns_name]
    }
    conf['vns']['configuration_name'] = vns_name

    # Call the planner
    pl = Planner(env, ignitions, scenario.flights, conf)
    res = pl.compute_plan()
    plan = res.final_plan()

    # Representation of unburned cells using max double is not suitable for display,
    # so those values must be converted to NaN
    ignitions_nan = ignitions['ignition'].copy()
    ignitions_nan[ignitions_nan == _DBL_MAX] = np.nan
    first_ignition = np.nanmin(ignitions_nan)
    last_ignition = np.nanmax(ignitions_nan)

    # Create the geodatadisplay object & extensions that are going to be used
    geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
    TrajectoryDisplayExtension(None).extend(geodatadisplay)

    plot_plan_with_background(pl, geodatadisplay, (first_ignition, last_ignition),
                              output_options_plot)

    # Save the picture
    print("saving as: " + str(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png')))))
    geodatadisplay.axis.get_figure().set_size_inches(*output_options_plot.get('size', (15, 10)))
    geodatadisplay.axis.get_figure().savefig(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png'))),
        dpi=output_options_plot.get('dpi', 150), bbox_inches='tight')

    # Log planned trajectories metadata
    for i, t in enumerate(plan.trajectories()):
        logging.info("traj #{} duration: {} / {}".format(i, t.duration(), t.conf.max_flight_time))

    # save metadata to file
    with open(os.path.join(save_directory, instance_name+".json"), "w") as metadata_file:
        import json
        parsed = json.loads(res.metadata())
        parsed["benchmark_id"] = instance_name
        parsed["date"] = time.strftime("%Y-%m-%d--%H:%M:%S")
        print(json.dumps(parsed, indent=4), file=metadata_file)

    matplotlib.pyplot.close(geodatadisplay.axis.get_figure())


    # If intermediate plans are available, save them
    for i in range(len(res.intermediate_plans)):
        geodatadisplay = GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        TrajectoryDisplayExtension(None).extend(geodatadisplay)
        plot_plan_with_background(pl, geodatadisplay, (first_ignition, last_ignition),
                                  output_options_plot, plan=i)

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
        uav_start = start + 3000
        max_flight_time = 450
        uav = UAVConf(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, max_flight_time)
        flights.append(FlightConf(uav, uav_start, random.choice(uav_bases)))

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
        uav_start = random.uniform(start + 2500, start + 7500.)
        max_flight_time = random.uniform(1000, 1500)
        uav = UAVConf(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, max_flight_time)
        flights.append(FlightConf(uav, uav_start, random.choice(uav_bases)))

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
        uav_start = random.uniform(start + 5000, start + 7000.)
        max_flight_time = random.uniform(500, 1000)
        uav = UAVConf(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, max_flight_time)
        flights.append(FlightConf(uav, uav_start, random.choice(uav_bases)))

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
        uav = UAVConf(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, max_flight_time)
        flights.append(FlightConf(uav, uav_start, random.choice(uav_bases)))

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
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        uav = UAVConf(uav_speed, uav_max_turn_rate, uav_max_pitch_angle, max_flight_time)
        flights.append(FlightConf(uav, uav_start, random.choice(uav_bases)))

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
    "demo": {
        "max_restarts": 0,
        "max_time": max_planning_time,
        "neighborhoods": [
            {"name": "dubins-opt",
                "max_trials": 10,
                "generators": [
                    {"name": "MeanOrientationChangeGenerator"},
                    {"name": "RandomOrientationChangeGenerator"},
                    {"name": "FlipOrientationChangeGenerator"}]},
            {"name": "one-insert",
                "max_trials": 50,
                "select_arbitrary_trajectory": False,
                "select_arbitrary_position": False}
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
    parser.add_argument("--name",
                        help="name of the benchmark. The resulting folder name will be prefixed by 'benchmark_'.",
                        choices=scenario_factory_funcs.keys())
    parser.add_argument("--folder",
                        help="Name of the folder in which benchmarks are to be saved.",
                        default=DEFAULT_FIRERS_DATA_FOLDER)
    parser.add_argument("--background", nargs='+',
                        help="List of background layers for the output figures, from bottom to top.",
                        choices=['elevation_shade', 'elevation_planning_shade', 'ignition_shade', 'observedcells', 'ignition_contour', 'wind_quiver'],
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
                        help="Load VNS configurations from a JSON file. The full path must be used.")
    parser.add_argument("--vns",
                        help="Select a VNS configuration, among the default ones or from the file specified in --vns-conf.",
                        default='demo')
    parser.add_argument("--elevation",
                        help="Source of elevation considered by the planning algorithm",
                        choices=['flat', 'dem', 'discrete'],
                        default='dem')
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

    if args.vns_conf:
        global vns_configurations
        vns_configurations = args.vns_conf

    # Set-up output options
    output_options = {'plot':{}, 'planning':{}, }
    output_options['plot']['background'] = args.background
    output_options['plot']['format'] = args.format
    output_options['plot']['colorbar'] = args.colorbar
    output_options['plot']['dpi'] = args.dpi
    output_options['plot']['size'] = args.size
    output_options['planning']['elevation_mode'] = args.elevation
    output_options['planning']['discrete_elevation_interval'] = \
        DISCRETE_ELEVATION_INTERVAL if args.elevation == 'discrete' else 0

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
        scenarios = [scenario_generator() for i in range(40)]
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
