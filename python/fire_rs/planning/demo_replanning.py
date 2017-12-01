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
    import fire_rs.geodata.display
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

    # First plan
    pl = Planner(env, fire.ignitions(), [fgconf], conf)
    pl.compute_plan()
    sr_1 = pl.search_result
    ei_1 = pl.expected_ignited_map(layer_name="expected_ignited")
    eo_1 = pl.expected_observed_map(layer_name="expected_observed")
    gdd = fire_rs.geodata.display.GeoDataDisplay(
        *fire_rs.geodata.display.get_pyplot_figure_and_axis(),
        env.raster.combine(fire.ignitions()).combine(ei_1).combine(eo_1))
    TrajectoryDisplayExtension(None).extend(gdd)
    gdd.draw_ignition_contour()
    gdd.draw_observation_map(layer='expected_observed', color='darkgreen', alpha=0.5)
    gdd.draw_observation_map(layer='expected_ignited', color='darkred', alpha=0.5)
    plot_plan_trajectories(sr_1.final_plan(), gdd, colors=["maroon"], show=True)

    # Re-plan from the part of the first plan
    from_t = fgconf.start_time + 5 * 60
    ei_1 = pl.expected_ignited_map((start_t, from_t + 1), layer_name="expected_ignited")
    eo_1 = pl.expected_observed_map((start_t, from_t + 1), layer_name="expected_observed")

    # With different wind
    fire_old  = fire.ignitions().clone(data_array=fire.ignitions()["ignition"], dtype=[('ignition_old', 'float64')])
    env.update_area_wind(wind[0]+1, wind[1])

    ignition_point = []
    for x in range(fire_old.max_x):
        for y in range(fire_old.max_y):
            t = pl.firemap["ignition"][x, y]
            if t <= from_t:
                p = pl.firemap.coordinates((x, y))
                ignition_point.append(TimedPoint(p[0], p[1], t))

    fire2 = propagation.propagate_from_points(env, ignition_point, 180 * 60)
    pl.update_firemap(fire2.ignitions())

    fire_difference = fire_old.clone(data_array=np.abs(fire_old['ignition_old']-fire2.ignitions()['ignition']), dtype=[('difference', 'float64')])

    sr_2 = pl.replan(from_t)
    ei_2 = pl.expected_ignited_map(layer_name="expected_ignited")
    eo_2 = pl.expected_observed_map(layer_name="expected_observed")

    gdd = fire_rs.geodata.display.GeoDataDisplay(
        *fire_rs.geodata.display.get_pyplot_figure_and_axis(),
        env.raster.combine(fire2.ignitions()).combine(fire_old))
    TrajectoryDisplayExtension(None).extend(gdd)
    gdd.draw_ignition_contour()
    gdd.draw_ignition_contour(layer="ignition_old")
    gdd.draw_ignition_shade(geodata=fire_difference, layer="difference", cmap=matplotlib.cm.Oranges)
    gdd.draw_observation_map(eo_1, layer='expected_observed', color='green', alpha=0.5)
    gdd.draw_observation_map(ei_1, layer='expected_ignited', color='red', alpha=0.5)
    gdd.draw_observation_map(eo_2, layer='expected_observed', color='darkgreen', alpha=0.5)
    gdd.draw_observation_map(ei_2, layer='expected_ignited', color='darkred', alpha=0.5)
    plot_plan_trajectories(sr_1.final_plan(), gdd, time_range=(start_t, from_t+1),
                           colors=["maroon"], show=False)
    plot_plan_trajectories(sr_2.final_plan(), gdd, colors=["orangered"], show=True)

    print(pl.search_result)