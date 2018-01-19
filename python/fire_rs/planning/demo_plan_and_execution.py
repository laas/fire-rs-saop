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

    from fire_rs.neptus_interface import upload_plan, start_plan, set_wind

    # Geographic environment (elevation, landcover, wind...)
    wind = (10., 0.)
    area = ((480060.0, 485060.0), (6210074.0, 6215074.0))
    flat_altitude = 700.
    env = PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                              planning_elevation_mode='flat', flat_altitude=flat_altitude)

    # Fire applied to the previous environment
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire = propagation.propagate_from_points(env, ignition_point, 180 * 60)

    # Configure some flight
    base_wp = Waypoint(area[0][0] + 100., area[1][0] + 100., flat_altitude + 100., 0.)
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
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
    gdd.draw_ignition_contour()
    gdd.draw_observation_map(layer='expected_observed', color='darkgreen', alpha=0.5)
    gdd.draw_observation_map(layer='expected_ignited', color='darkred', alpha=0.5)
    plot_plan_trajectories(sr_1.final_plan(), gdd, colors=["maroon"], show=True)

    upload_plan("127.0.0.1", "6002", sr_1.final_plan(), "Fire plan long obs sampled(-1)", 50., -1)
    start_plan("127.0.0.1", "6002", "Fire plan long obs sampled(-1)")
    set_wind("127.0.0.1", "6002", *wind)

    print("Expected duration: " + str(sr_1.final_plan().duration()))
    print(" - - END - - ")
