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


_DBL_MAX = np.finfo(np.float64).max

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
        geodatadisplay.draw_path = types.MethodType(PlanDisplayExtension._draw_path_extension, geodatadisplay)
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

    def _draw_path_extension(self, *args, single_color: 'Optional[str]' = None,
                             colorbar_time_range: 'Optional[Tuple[float, float]]' = None, **kwargs):
        '''Draw trajectory in a GeoDataDisplay figure.

        If the single_color argument is set, the trajectory will be displayed with a single color and
        other color related options are ignored.

        Otherwise the color of the trajectory will reflect the time taken by the trajectory.
        Optional argument colorbar_time_range may be a tuple of start and end times in seconds.
        If the Optional argument with_colorbar is set to True, a color bar will be displayed of the trajectory.
        '''
        if len(self.plan_trajectory.segments) < 2:
            return
        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        if single_color:
            self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=single_color, zorder=2))
        else:
            color_range = np.linspace(self.plan_trajectory.start_time() / 60, self.plan_trajectory.end_time() / 60, len(x))
            color_norm = matplotlib.colors.Normalize(vmin=color_range[0], vmax=color_range[-1])
            if colorbar_time_range is not None:
                color_norm = matplotlib.colors.Normalize(vmin=colorbar_time_range[0]/60, vmax=colorbar_time_range[1]/60)
            self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=color_range,
                                       norm=color_norm, cmap=matplotlib.cm.gist_rainbow, zorder=2))
            if kwargs.get('with_colorbar', False):
                cb = self._figure.colorbar(self._drawings[-1], ax=self.axis, shrink=0.65, aspect=20)
                cb.set_label("Flight time [min]")
                self._colorbars.append(cb)

    def _draw_segments_extension(self, *args, **kwargs):
        '''Draw path segments in a GeoDataDisplay figure.'''
        if len(self.plan_trajectory.segments) < 2:
            return
        color = kwargs.get('color', 'C0')
        segments = self.plan_trajectory.segments[1:-1]
        start_x = [s.start.x for s in segments]
        start_y = [s.start.y for s in segments]
        end_x = [s.end.x for s in segments]
        end_y = [s.end.y for s in segments]
        self._drawings.append(self.axis.scatter(start_x, start_y, s=10, edgecolor='black', c=color, marker='D', zorder=2))
        self._drawings.append(self.axis.scatter(end_x, end_y, s=10, edgecolor='black', c=color, marker='>', zorder=2))

        start_base = self.plan_trajectory.segments[0]
        finish_base = self.plan_trajectory.segments[-1]

        self._drawings.append(
            self.axis.scatter(start_base.start.x, start_base.start.y, s=10, edgecolor='black', c=color, marker='o', zorder=2))
        self._drawings.append(
            self.axis.scatter(finish_base.start.x, finish_base.start.y, s=10, edgecolor='black', c=color, marker='o', zorder=2))

    def _draw_observedcells(self, observations, **kwargs):
        for ptt in observations:
            self.axis.scatter(ptt.as_tuple()[0][0], ptt.as_tuple()[0][1], s=4, c=(0., 1., 0., .5),
                              zorder=2, edgecolors='none', marker='s')


def plot_plan(plan, geodatadisplay, time_range: 'Optional[Tuple[float, float]]' = None, show=False):
    colors = cycle(["red", "green", "blue", "black", "magenta"])
    for traj, color in zip(plan.trajectories, colors):
        geodatadisplay.plan_trajectory = traj
        geodatadisplay.draw_path(single_color=color)
        # geodatadisplay.draw_path(colorbar_time_range=time_range, with_colorbar=True)
        geodatadisplay.draw_segments(color=color)
    if show:
        geodatadisplay.axis.get_figure().show()


def run_benchmark(scenario, save_directory, instance_name, output_options_plot: dict, snapshots, plot=False):
    import os
    from fire_rs.firemodel import propagation
    env = propagation.Environment(scenario.area, wind_speed=scenario.wind_speed, wind_dir=scenario.wind_direction)
    prop = propagation.propagate_from_points(env, scenario.ignitions, horizon=scenario.time_window_end + 60 * 10)

    # Create and run a plan for this scenario
    ignitions = prop.ignitions()

    # Propagation was let running more time than desired in order to reduce edge effect.
    # Now we need to filter out-of-range ignition times. Crop range is rounded up to the next 10-minute mark (minus 1).
    ignitions['ignition'][ignitions['ignition'] > int(scenario.time_window_end / 60. + 5) * 60. - 60] = _DBL_MAX
    flights = [f.as_trajectory_config() for f in scenario.flights]
    # ax = ignitions.plot(blocking=False)
    res = up.plan_vns(flights, ignitions.as_cpp_raster(), env.raster.slice('elevation').as_cpp_raster(),
                      scenario.time_window_start, scenario.time_window_end, save_every=0, save_improvements=snapshots,
                      discrete_elevation_interval=100)
    plan = res.final_plan()


    # Representation of unburned cells using max double is not suitable for display,
    # so those values must be converted to NaN
    ignitions_nan = ignitions['ignition'].copy()
    ignitions_nan[ignitions_nan == _DBL_MAX] = np.nan
    first_ignition = np.nanmin(ignitions_nan)
    last_ignition = np.nanmax(ignitions_nan)

    # Draw the final plan on a figure
    geodatadisplay = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
    PlanDisplayExtension(None).extend(geodatadisplay)

    for layer in output_options_plot['background']:
        if layer == 'elevation_shade':
            geodatadisplay.draw_elevation_shade(with_colorbar=True)
        elif layer == 'ignition_shade':
            geodatadisplay.draw_ignition_shade(with_colorbar=True)
        elif layer == 'observedcells':
            geodatadisplay.draw_observedcells(res.final_plan().observations())
        elif layer == 'ignition_contour':
            geodatadisplay.draw_ignition_contour(with_labels=True)
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()

    for i, t in enumerate(res.final_plan().trajectories):
        print("traj #{} duration: {} / {}".format(i, t.duration(), t.conf.max_flight_time))

    plot_plan(res.final_plan(), geodatadisplay, time_range=(first_ignition, last_ignition), show=True)
    print("saving as: " + str(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png')))))
    geodatadisplay.axis.get_figure().set_size_inches(20, 15)
    geodatadisplay.axis.get_figure().savefig(os.path.join(
        save_directory, instance_name + "." + str(output_options_plot.get('format', 'png'))),
        dpi=output_options_plot.get('dpi', 150), bbox_inches='tight')

    matplotlib.pyplot.close(geodatadisplay.axis.get_figure())


    # Save intermediate plans
    for i, i_plan in enumerate(res.intermediate_plans):
        geodatadisplay = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(ignitions))
        PlanDisplayExtension(None).extend(geodatadisplay)
        for layer in output_options_plot['background']:
            if layer == 'elevation_shade':
                geodatadisplay.draw_elevation_shade(with_colorbar=True)
            elif layer == 'ignition_shade':
                geodatadisplay.draw_ignition_shade(with_colorbar=True)
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
        geodatadisplay.axis.get_figure().set_size_inches(20, 15)
        geodatadisplay.axis.get_figure().savefig(os.path.join(
            i_plan_dir, str(i) + "." + str(output_options_plot.get('format', 'png'))),
            dpi=output_options_plot.get('dpi', 150), bbox_inches='tight')
        matplotlib.pyplot.close(geodatadisplay.axis.get_figure())

    # Save plan statistics
    summary_file = os.path.join(save_directory, "summary.csv")
    if not os.path.exists(summary_file):
        with open(summary_file, "a") as summary:
            summary.write("instance,planning-time,preprocessing-time,utility,duration\n")

    with open(summary_file, "a") as summary:
        summary.write("{},{},{},{},{}\n".format(
            instance_name, res.planning_time, res.preprocessing_time, plan.utility(), plan.duration()))

    del res


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
        max_flight_time = random.uniform(1000, 2000)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


scenario_factory_funcs = {'default': generate_scenario,
                          'singlefire_singleuav': generate_scenario_singlefire_singleuav,
                          'singlefire_singleuav_shortrange': generate_scenario_singlefire_singleuav_shortrange,
                          'singlefire_singleuav_3d': generate_scenario_singlefire_singleuav_3d,}


def main():
    import pickle
    import os
    import argparse
    import logging
    from fire_rs.geodata.environment import DEFAULT_FIRERS_DATA_FOLDER

    # CLI argument parsing
    parser = argparse.ArgumentParser(prog='benchmark.py')
    parser.add_argument("--name",
                        help="name of the benchmark. The resulting folder name will be prefixed by 'benchmark_'.")
    parser.add_argument("--background", nargs='+',
                        help="list of background layers for the output figures, from bottom to top.",
                        choices=['elevation_shade', 'ignition_shade', 'observedcells', 'ignition_contour', 'wind_quiver'],
                        default=['elevation_shade', 'ignition_contour', 'wind_quiver'])
    parser.add_argument("--format",
                        help="format of the output figures",
                        choices=['png', 'svg'],
                        default='png')
    parser.add_argument("--dpi",
                        help="resolution of the output figures",
                        type=int,
                        default=150)
    parser.add_argument("--instance",
                        help="Runs a particular benchmark instance",
                        type=int,
                        default=None)
    parser.add_argument("--snapshots", action="store_true",
                        help="save snapshots of the plan after every improvement. Beware, this option will slowdown the simulation and consume lots of memory",
                        default=False)
    parser.add_argument("--parallel", action="store_true",
                        help="enable parallel scenario processing")
    parser.add_argument("--wait", action="store_true",
                        help="wait for user input before start. Useful to hold the execution while attaching to a debugger")
    args = parser.parse_args()

    # Set-up output options
    output_options = {'plot':{},}
    output_options['plot']['background'] = args.background
    output_options['plot']['format'] = args.format
    output_options['plot']['dpi'] = args.dpi

    # benchmark folder handling
    benchmark_name = args.name
    benchmark_name_full = "benchmark"
    if benchmark_name:
        benchmark_name_full = "_".join([benchmark_name_full, str(benchmark_name)])
    benchmark_dir = os.path.join(DEFAULT_FIRERS_DATA_FOLDER, benchmark_name_full)
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

    if args.parallel:
        import joblib
        ALL_CPU_BUT_ONE = -2
        joblib.Parallel(n_jobs=4, backend="threading", verbose=5)\
            (joblib.delayed(run_benchmark)(s, run_dir, str(i), output_options_plot=output_options['plot'],
                                           snapshots=args.snapshots) for i, s in to_run)
    else:
        for i, scenario in to_run:
            print(scenario)
            run_benchmark(scenario, run_dir, str(i), output_options_plot=output_options['plot'], snapshots=args.snapshots)


if __name__=='__main__':
    main()
