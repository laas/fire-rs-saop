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


if __name__ == '__main__':
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
    area = ((480060.0, 485060.0), (6210074.0, 6215074.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=0)

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire = propagation.propagate_from_points(env, ignition_point, 240 * 60)

    # Configure some flight
    base_wp = Waypoint(area[0][0] + 100., area[1][0] + 100., 0., 0.)
    start_t = 180 * 60  # 30 minutes after the ignition
    uavconf = UAVConf.X8()
    uavconf.max_flight_time = 1000
    fgconf = FlightConf(uavconf, start_t, base_wp)

    # Write down the desired VNS configuration
    conf_vns = {
        "full": {
            "max_restarts": 5,
            "max_time": 30.0,
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

    # 2nd PLAN
    from_t_2 = fgconf.start_time + 300
    fire2 = fire.ignitions().clone(data_array=fire.ignitions()["ignition"] + 600,
                                   dtype=[('ignition', 'float64')])
    pl.update_firemap(fire2)
    sr_2 = pl.replan_after_time(from_t_2)

    # 3rd PLAN
    from_t_3 = fgconf.start_time + 600
    fire3 = fire.ignitions().clone(data_array=fire.ignitions()["ignition"] - 600,
                                   dtype=[('ignition', 'float64')])
    pl.update_firemap(fire3)
    sr_3 = pl.replan_after_time(from_t_3)

    fmapper = FireMapper(env, fire3)  # fire3 is the ground thruth

    ####################################
    # 1st figure: show only the first plan
    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire3))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_3.final_plan().trajectories()[0].start_time(0) - 120,
               sr_3.final_plan().trajectories()[0].end_time(len(
                   sr_3.final_plan().trajectories()[0]) - 1) + 120)
    gdd.draw_ignition_contour(geodata=fire1, time_range=t_range, cmap=matplotlib.cm.Blues)

    # Draw observed fire
    executed_path_1 = sr_1.final_plan().trajectories()[0].sampled_with_time((0, from_t_2),
                                                                            step_size=10)
    fmapper.observe(executed_path_1, pl.flights[0].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.viridis,
                            vmin=t_range[0], vmax=t_range[1])

    # Draw trajectory
    plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=['blue'], labels=["1st plan"], linestyles=[':'])
    plot_plan_trajectories(sr_1.final_plan(), gdd, time_range=(0, from_t_2 - 60), trajectories=0,
                           colors=['blue'], labels=["executed"], linestyles=['-'])
    gdd.legend()
    gdd.figure.show()

    ####################################
    # 2nd figure: First and second plan
    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire3))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_3.final_plan().trajectories()[0].start_time(0) - 120,
               sr_3.final_plan().trajectories()[0].end_time(len(
                   sr_3.final_plan().trajectories()[0]) - 1) + 120)
    gdd.draw_ignition_contour(geodata=fire2, time_range=t_range, cmap=matplotlib.cm.Greens)

    # Draw observed fire
    executed_path_2 = sr_2.final_plan().trajectories()[0].sampled_with_time((from_t_2, from_t_3),
                                                                            step_size=10)
    fmapper.observe(executed_path_2, pl.flights[0].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.viridis,
                            vmin=t_range[0], vmax=t_range[1])
    # Draw trajectory planned
    plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=[(0., 0., 1., .33)], labels=["1st plan"], linestyles=[':'])
    plot_plan_trajectories(sr_2.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=['green'], labels=["2nd plan"], linestyles=['--'])

    # Draw trajectory executed
    replan_id = sr_2.final_plan().trajectories()[0].first_modifiable_id() - 1
    replan_time = sr_2.final_plan().trajectories()[0].start_time(replan_id)
    plot_plan_trajectories(sr_1.final_plan(), gdd, time_range=(0, replan_time), trajectories=0,
                           colors=['blue'], labels=["1st executed"], linestyles=['-'])
    plot_plan_trajectories(sr_2.final_plan(), gdd, time_range=(replan_time, from_t_3),
                           trajectories=0, colors=['green'], labels=["2nd executed"],
                           linestyles=['-'])
    gdd.legend()
    gdd.figure.show()

    ####################################
    # 3rd figure: First, second plan and third plan
    gdd = fire_rs.geodata.display.GeoDataDisplay.pyplot_figure(env.raster.combine(fire3))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})

    # Draw expected fire contour
    t_range = (sr_3.final_plan().trajectories()[0].start_time(0) - 120,
               sr_3.final_plan().trajectories()[0].end_time(len(
                   sr_3.final_plan().trajectories()[0]) - 1) + 120)
    gdd.draw_ignition_contour(geodata=fire3, time_range=t_range, cmap=matplotlib.cm.Reds)

    # Draw observed fire
    replan_id_3 = sr_3.final_plan().trajectories()[0].first_modifiable_id() - 1
    replan_time_3 = sr_3.final_plan().trajectories()[0].start_time(replan_id)
    executed_path_3 = sr_3.final_plan().trajectories()[0].sampled_with_time((replan_time_3, np.inf),
                                                                            step_size=10)
    fmapper.observe(executed_path_3, pl.flights[0].uav)
    gdd.draw_ignition_shade(geodata=fmapper.firemap, cmap=matplotlib.cm.viridis,
                            vmin=t_range[0], vmax=t_range[1])

    # Draw trajectory planned
    plot_plan_trajectories(sr_1.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=[(0., 0., 1., .33)], labels=["1st plan"], linestyles=[':'])
    plot_plan_trajectories(sr_2.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=[(0., 1., 0., .33)], labels=["2nd plan"], linestyles=[':'])
    plot_plan_trajectories(sr_3.final_plan(), gdd, trajectories=0, draw_segments=False,
                           colors=['red'], labels=["3rd plan"], linestyles=['--'])

    # Draw trajectory executed

    plot_plan_trajectories(sr_3.final_plan(), gdd, time_range=(0, np.inf),
                           trajectories=0, colors=['red'], labels=["3rd executed"],
                           linestyles=['-'])
    plot_plan_trajectories(sr_2.final_plan(), gdd, time_range=(0, replan_time_3),
                           trajectories=0, colors=['green'], labels=["2nd executed"],
                           linestyles=['-'])
    plot_plan_trajectories(sr_1.final_plan(), gdd, time_range=(0, replan_time), trajectories=0,
                           colors=['blue'], labels=["1st executed"], linestyles=['-'])
    gdd.legend()
    gdd.figure.show()

    print(pl.search_result)
