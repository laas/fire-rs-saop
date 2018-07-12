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

"""This demo shows a quite slow UAV doing some planned trajectory, depicting the
non-stationarity property of the observation task"""

if __name__ == '__main__':
    import logging

    logging.basicConfig(level=logging.DEBUG)

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

    # Geographic environment (elevation, landcover, wind...)
    wind = (10., 0.)
    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    logging.info("Start of propagation")
    fire = propagation.propagate_from_points(env, ignition_point, 240 * 60)
    logging.info("End of propagation")

    # Configure some flight
    base_wp = Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.)
    start_t = 120 * 60  # 30 minutes after the ignition
    uavconf = UAVConf.slow_x8()
    fgconf = FlightConf(uavconf, start_t, base_wp)

    # Write down the desired VNS configuration
    conf_vns = {
        "full": {
            "max_restarts": 5,
            "max_time": 15.0,
            "neighborhoods": [
                {"name": "dubins-opt",
                 "max_trials": 100,
                 "generators": [
                     {"name": "MeanOrientationChangeGenerator"},
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
        }
    }

    conf = {
        'min_time': fgconf.start_time,
        'max_time': fgconf.start_time + fgconf.uav.max_flight_time,
        'save_every': 0,
        'save_improvements': False,
        'discrete_elevation_interval': 0,
        'vns': conf_vns['full']
    }
    conf['vns']['configuration_name'] = 'full'

    ####################################
    # 1st PLAN
    fire1 = fire.ignitions()
    pl = Planner(env, fire1, [fgconf], conf)
    pl.compute_plan()
    sr_1 = pl.search_result

    fmapper = FireMapper(env, fire1)  # fire3 is the ground thruth

    ####################################
    # 1st figure: show only the first plan
    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire1),
                                                               frame=(0, 0))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_1.final_plan().trajectories()[0].start_time(0) - 120,
               sr_1.final_plan().trajectories()[0].end_time(len(
                   sr_1.final_plan().trajectories()[0]) - 1) + 120)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range, cmap=matplotlib.cm.Reds)

    # Draw observed fire
    # executed_path_1 = sr_1.final_plan().trajectories()[0].sampled_with_time(step_size=10)
    # fmapper.observe(executed_path_1, pl.flights[0].uav)
    # gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.viridis,
    #                         vmin=t_range[0], vmax=t_range[1])

    # Draw trajectory
    plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=0, draw_path=False,
                           draw_flighttime_path=True, colors=['blue'], labels=["executed"],
                           linestyles=['-'])
    gdd.legend()
    gdd.figure.show()

    print(pl.search_result)
