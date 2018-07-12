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

"""This demo draws result figures for the winter2017 paper"""

import logging

import numpy as np
import matplotlib
import matplotlib.cm

import fire_rs.uav_planning as op
import fire_rs.neptus_interface as neptus
import fire_rs.firemapping as fmapping

import fire_rs.geodata.display
from fire_rs.firemodel import propagation
from fire_rs.geodata.geo_data import TimedPoint, GeoData
from fire_rs.planning.planning import FireMapper, FlightConf, Planner, PlanningEnvironment, \
    UAVConf, Waypoint
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories

logging.basicConfig(level=logging.DEBUG)


def SAOP_conf(min_time: float, max_time: float) -> 'dict':
    # Write down the desired VNS configuration
    conf_vns = {
        "full": {
            "max_restarts": 5,
            "max_time": 60.0,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 100,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
                     {"name": "RandomOrientationChangeGenerator"},
                     {"name": "FlipOrientationChangeGenerator"}]},
                {"name": "one-insert",
                 "max_trials": 50,
                 "select_arbitrary_trajectory": True,
                 "select_arbitrary_position": False},
            ]
        }
    }

    conf = {
        'min_time': min_time,
        'max_time': max_time,
        'save_every': 0,
        'save_improvements': False,
        'discrete_elevation_interval': 0,
        'vns': conf_vns['full']
    }
    conf['vns']['configuration_name'] = 'full'

    return conf


def wp_insertion() -> "fire_rs.geodata.display.GeoDataDisplay":
    """Plot a case with an small fire. Overlay arrows indicating fire propagation direction"""

    # Geographic environment (elevation, landcover, wind...)
    wind = (2., 0.)
    area = ((480000.0, 480200.0), (6210000.0, 6210200.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    ignition_points = [
        TimedPoint(area[0][0], area[1][0], 0),
    ]
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_points, 180 * 60)
    logging.info("End of propagation")

    fire1 = fire.ignitions()

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               )
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # print(env.raster.x_offset)
    # gdd.axis.set_xticks(np.arange(area[0][0]-25, area[0][1], 22.22))
    # gdd.axis.set_yticks(np.arange(area[1][0]-25, area[1][1], 22.22))
    # gdd.axis.grid(True)
    gdd.axes.tick_params(
        axis='both',  # changes apply to the x-axis
        which='both',  # both major and minor ticks are affected
        bottom='off',  # ticks along the bottom edge are off
        left='off',  # ticks along the bottom edge are off
        top='off',  # ticks along the top edge are off
        labelleft='off',  # ticks along the bottom edge are off
        labelbottom='off')  # labels along the bottom edge are off
    gdd.axes.set_xlabel("")
    gdd.axes.set_ylabel("")
    t_range_fire = (0, np.inf)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range_fire, cmap=matplotlib.cm.plasma)
    gdd.draw_ignition_shade(with_colorbar=False, geodata=fire1, vmin=0, vmax=120*60, cmap=matplotlib.cm.Reds)

    gdd.legend()
    return gdd


def detail_case_figure(wind_speed=5., wind_dir=0.) -> "fire_rs.geodata.display.GeoDataDisplay":
    """Plot a case with an small fire
     we can make a zoom and observe a dubinswind path and observed cells."""

    # Geographic environment (elevation, landcover, wind...)
    wind = (wind_speed, wind_dir)
    wind_xy = (wind[0] * np.cos(wind[1]), wind[0] * np.sin(wind[1]))
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=10, wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_points = [
        TimedPoint(area[0][0] + 2000.0, area[1][0] + 2000.0, 0),
    ]
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_points, 180 * 60)
    logging.info("End of propagation")

    # Configure some flight
    base_wp = Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.)
    start_t = 120 * 60
    uav = UAVConf.x8()

    f_confs = [
        FlightConf(uav, start_t, Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.), None,
                   wind_xy),
    ]

    conf = SAOP_conf(start_t, start_t + uav.max_flight_time)

    fire1 = fire.ignitions()
    pl = Planner(env, fire1, f_confs, conf)
    pl.compute_plan()
    sr_1 = pl.search_result
    fmapper = FireMapper(env, fire1)

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    gdd.axes.grid(True)

    # Draw expected fire contour
    t_range = (sr_1.final_plan().trajectories()[0].start_time(0) - 120,
               sr_1.final_plan().trajectories()[0].end_time(len(
                   sr_1.final_plan().trajectories()[0]) - 1) + 120)
    t_range_fire = (0, np.inf)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range_fire, cmap=matplotlib.cm.Reds)

    # Draw observed fire
    executed_path_1 = sr_1.final_plan().trajectories()[0].sampled_with_time(step_size=10)
    fmapper.observe(executed_path_1, pl.flights[0].uav)
    gdd.TrajectoryDisplayExtension.draw_observation_map(obs_map=fmapper.observed, layer='ignition',
                                                        color='gray')
    gdd.TrajectoryDisplayExtension.draw_observation_map(obs_map=fmapper.firemap, layer='ignition',
                                                        color='green')

    # Draw trajectory
    colors = ['blue', 'green', 'magenta']
    labels = ["UAV " + str(i) for i in range(len(f_confs))]

    for i in range(len(f_confs)):
        plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=i, draw_path=True,
                               draw_segments=False, draw_flighttime_path=False, colors=[colors[i]],
                               labels=None)
    gdd.legend()
    return gdd


def singleuav_case_figure(wind_speed=5., wind_dir=0.,
                          uav_w_speed=None) -> "fire_rs.geodata.display.GeoDataDisplay":
    """Plot a case with wind, 1 UAV, 1 fire."""

    # Geographic environment (elevation, landcover, wind...)
    uav_w_speed = wind_speed if uav_w_speed is None else uav_w_speed
    wind = (uav_w_speed, wind_dir)
    wind_xy = (wind[0] * np.cos(wind[1]), wind[0] * np.sin(wind[1]))
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=wind[0] * 2, wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_points = [
        TimedPoint(area[0][0] + 2000.0, area[1][0] + 2000.0, 0),
    ]
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_points, 180 * 60)
    logging.info("End of propagation")

    # Configure some flight
    base_wp = Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.)
    start_t = 120 * 60
    uav = UAVConf.x8()

    f_confs = [
        FlightConf(uav, start_t, Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.), None,
                   wind_xy),
    ]

    conf = SAOP_conf(start_t, start_t + uav.max_flight_time)

    fire1 = fire.ignitions()
    pl = Planner(env, fire1, f_confs, conf)
    pl.compute_plan()
    sr_1 = pl.search_result
    fmapper = FireMapper(env, fire1)

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_1.final_plan().trajectories()[0].start_time(0) - 120,
               sr_1.final_plan().trajectories()[0].end_time(len(
                   sr_1.final_plan().trajectories()[0]) - 1) + 120)
    t_range_fire = (0, np.inf)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range_fire, cmap=matplotlib.cm.Reds)

    # Draw observed fire
    executed_path_1 = sr_1.final_plan().trajectories()[0].sampled_with_time(step_size=10)
    fmapper.observe(executed_path_1, pl.flights[0].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.summer, with_colorbar=False)

    # Draw trajectory
    colors = ['blue', 'green', 'magenta']
    labels = ["UAV " + str(i) for i in range(len(f_confs))]

    for i in range(len(f_confs)):
        plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=i, draw_path=True,
                               draw_flighttime_path=False, colors=[colors[i]], labels=[labels[i]],
                               linestyles=['-'])
    gdd.legend()
    return gdd


def generic_case_figure(wind_speed=5.,
                        wind_dir=np.pi / 2) -> "fire_rs.geodata.display.GeoDataDisplay":
    """Plot a generic case with wind, multiple UAV from multiple bases, and multiple fire."""

    # Geographic environment (elevation, landcover, wind...)
    wind = (wind_speed, wind_dir)
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_points = [
        TimedPoint(area[0][0] + 1000.0, area[1][0] + 1000.0, 0.),
        TimedPoint(area[0][0] + 4000.0, area[1][0] + 3000.0, 0.)
    ]
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_points, 120 * 60)
    logging.info("End of propagation")

    # Configure some flight
    start_t = 90 * 60
    uav = UAVConf.x8()
    uav.max_flight_time/=5
    f_confs = [
        FlightConf(uav, start_t, Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.), None,
                   wind),
        FlightConf(uav, start_t, Waypoint(area[0][1] - 100., area[1][0] + 1000., 0., 0.), None,
                   wind),
    ]

    conf = SAOP_conf(start_t, start_t + uav.max_flight_time)

    fire1 = fire.ignitions()
    pl = Planner(env, fire1, f_confs, conf)
    pl.compute_plan()
    sr_1 = pl.search_result
    fmapper = FireMapper(env, fire1)

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_1.final_plan().trajectories()[0].start_time(0) - 120,
               sr_1.final_plan().trajectories()[0].end_time(len(
                   sr_1.final_plan().trajectories()[0]) - 1) + 120)
    t_range_fire = (0, np.inf)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range_fire, cmap=matplotlib.cm.Reds)

    # Draw observed fire
    for i in range(len(f_confs)):
        executed_path = sr_1.final_plan().trajectories()[i].sampled_with_time(step_size=10)
        fmapper.observe(executed_path, pl.flights[i].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.summer,
                            vmin=t_range[0], vmax=t_range[1], with_colorbar=False)

    # Draw trajectory
    colors = ['blue', 'green', 'magenta']
    labels = ["UAV " + str(i) for i in range(len(f_confs))]

    for i in range(len(f_confs)):
        plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=i, draw_path=True,
                               draw_flighttime_path=False, colors=[colors[i]], labels=[labels[i]],
                               linestyles=['-'])
    gdd.legend()
    return gdd


def threefire_twouav_figure(wind_speed=5.,
                        wind_dir=np.pi / 2) -> "fire_rs.geodata.display.GeoDataDisplay":
    """Plot a generic case with wind, multiple UAV from multiple bases, and multiple fire."""

    # Geographic environment (elevation, landcover, wind...)
    wind = (wind_speed, wind_dir)
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_points = [
        TimedPoint(area[0][0] + 1000.0, area[1][0] + 1000.0, 0.),
        TimedPoint(area[0][0] + 4000.0, area[1][0] + 2000.0, 0.),
        TimedPoint(area[0][0] + 2000.0, area[1][0] + 3500.0, 0.)
    ]
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_points, 120 * 60)
    logging.info("End of propagation")

    # Configure some flight
    start_t = 90 * 60
    uav = UAVConf.x8()
    uav.max_flight_time /= 3
    f_confs = [
        FlightConf(uav, start_t, Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.), None,
                   wind),
        FlightConf(uav, start_t, Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.), None,
                   wind),
    ]

    conf = SAOP_conf(start_t, start_t + uav.max_flight_time)

    fire1 = fire.ignitions()
    pl = Planner(env, fire1, f_confs, conf)
    pl.compute_plan()
    sr_1 = pl.search_result
    fmapper = FireMapper(env, fire1)

    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_1.final_plan().trajectories()[0].start_time(0),
               sr_1.final_plan().trajectories()[0].end_time(len(
                   sr_1.final_plan().trajectories()[0]) - 1))
    t_range_fire = (0, np.inf)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range_fire, cmap=matplotlib.cm.Reds,
                              alpha=1)

    # Draw observed fire
    for i in range(len(f_confs)):
        executed_path = sr_1.final_plan().trajectories()[i].sampled_with_time(step_size=10)
        fmapper.observe(executed_path, pl.flights[i].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.summer,
                            vmin=t_range[0], vmax=t_range[1], with_colorbar=False)

    # Draw trajectory
    colors = ['blue', 'darkgreen', 'magenta']
    labels = ["UAV " + str(i) for i in range(len(f_confs))]

    for i in range(len(f_confs)):
        plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=i, draw_path=True,
                               draw_flighttime_path=False, colors=[colors[i]], labels=[labels[i]],
                               linestyles=['-'])
    gdd.legend()
    return gdd

def show(gdd, i):
    gdd.figure.set_size_inches(6, 4.5)
    gdd.figure.show()


def archive(gdd, i):
    gdd.figure.set_size_inches(6, 4.5)
    gdd.figure.savefig("result_" + str(i) + ".pdf", dpi=300, bbox_inches='tight')
    gdd.figure.savefig("result_" + str(i) + ".eps", dpi=300, bbox_inches='tight')
    gdd.figure.savefig("result_" + str(i) + ".svg", dpi=300, bbox_inches='tight')


if __name__ == '__main__':
    # input("Press any key...")

    matplotlib.rcParams['font.family'] = 'sans-serif'
    matplotlib.rcParams['text.usetex'] = True
    #
    # f1 = singleuav_case_figure()
    # show(f1, 1)
    # archive(f1, 1)
    #
    f2 = generic_case_figure()
    show(f2, 2 )
    # archive(f2, 2)
    #
    # f3 = singleuav_case_figure(wind_speed=2., wind_dir=np.pi/2)
    # archive(f3, 3)
    #
    # f4 = detail_case_figure()
    # show(f4, 4)
    # archive(f4, 4)
    #
    # f5 = wp_insertion()
    # show(f5, "wp_insertion")
    # archive(f5, "wp_insertion")

    # f6 = twofire_threeuav_figure(wind_speed=2., wind_dir=np.pi/2)
    # show(f6, "twofire_threeuav")
    # archive(f6, "twofire_threeuav")

    f7 = threefire_twouav_figure()
    show(f7, "threefire_twouav")
    archive(f7, "threefire_twouav")

    print("eee")
