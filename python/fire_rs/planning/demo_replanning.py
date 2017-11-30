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
    from fire_rs.firemodel import propagation
    from fire_rs.geodata.geo_data import TimedPoint
    from fire_rs.planning.planning import Planner, PlanningEnvironment, Waypoint, FlightConf, UAVConf
    from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories

    # Geographic environment (elevation, landcover, wind...)
    wind = (10., 0.)
    area = ((480060.0, 485060.0), (6210074.0, 6215074.0))
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat')

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire = propagation.propagate_from_points(env, ignition_point, 180 * 60)

    # Configure some flight
    base_wp = Waypoint(area[0][0]+100., area[1][0]+100., 100., 0.)
    start_t = 120 * 60  # 30 minutes after the ignition
    fgconf = FlightConf(UAVConf.X8(), start_t, base_wp)

    # Write down the desired VNS configuration
    conf_vns = {
        "demo": {
        "max_time": 15.,
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
    }
    }

    conf = {
        'min_time': fgconf.start_time,
        'max_time': fgconf.start_time + fgconf.uav.max_flight_time,
        'save_every': 0,
        'save_improvements': False,
        'discrete_elevation_interval': 0,
        'vns': conf_vns['demo']
    }
    conf['vns']['configuration_name'] = 'demo'

    # Instantiate the planner
    pl = Planner(env, fire.ignitions(), [fgconf], conf)

    pl.compute_plan()
    sr_1 = pl.search_result
    observ = pl.expected_ignited_positions((fgconf.start_time, fgconf.start_time + fgconf.uav.max_flight_time))
    fm_1 = pl.expected_ignited_map()

    import matplotlib
    import fire_rs.geodata.display
    gdd = fire_rs.geodata.display.GeoDataDisplay(
        *fire_rs.geodata.display.get_pyplot_figure_and_axis(), env.raster.combine(fm_1).combine(fire.ignitions()))
    TrajectoryDisplayExtension(None).extend(gdd)
    gdd.draw_ignition_contour()
    gdd.draw_observation_map(fm_1, color='darkgreen')
    plot_plan_trajectories(sr_1.final_plan(), gdd, colors=["maroon"], show=True)

    # Replan
    from_t = fgconf.start_time + 5 * 60
    fm_1 = pl.expected_observed_map((start_t, from_t + 1))

    # With different wind
    fire_old  = fire.ignitions().clone(data_array=fire.ignitions()["ignition"], dtype=[('ignition_old', 'float64')])
    env.update_area_wind(wind[0]+1, wind[1]-10/180*np.pi)
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire2 = propagation.propagate_from_points(env, ignition_point, 180 * 60)
    pl.update_firemap(fire2.ignitions())

    sr_2 = pl.replan(from_t)
    fm_2 = pl.expected_observed_map()

    gdd = fire_rs.geodata.display.GeoDataDisplay(
        *fire_rs.geodata.display.get_pyplot_figure_and_axis(),
        env.raster.combine(fm_2).combine(fire2.ignitions()).combine(fire_old))
    TrajectoryDisplayExtension(None).extend(gdd)
    gdd.draw_ignition_contour()
    gdd.draw_ignition_contour(layer="ignition_old")
    gdd.draw_observation_map(fm_2, color='chartreuse')
    gdd.draw_observation_map(fm_1, color='darkgreen')
    plot_plan_trajectories(sr_1.final_plan(), gdd, time_range=(start_t, from_t+1),
                           colors=["maroon"], show=False)
    plot_plan_trajectories(sr_2.final_plan(), gdd, colors=["orangered"], show=True)

    print(pl.search_result)