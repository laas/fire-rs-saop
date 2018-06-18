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

from typing import Sequence, Tuple

import matplotlib.cm as mcm

import fire_rs.uav_planning as op
import fire_rs.neptus_interface as neptus
import fire_rs.firemapping as fmapping

import fire_rs.firemodel.propagation as wildfire
import fire_rs.planning.planning as planning
import fire_rs.planning.benchmark as benchmark

from fire_rs.geodata.geo_data import GeoData
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories

if __name__ == "__main__":
    def get_initial_planning_environment(
            scenario: 'benchmark.Scenario') -> 'planning.PlanningEnvironment':
        env = planning.PlanningEnvironment(scenario.area, wind_speed=scenario.wind_speed,
                                           wind_dir=scenario.wind_direction,
                                           planning_elevation_mode='flat',
                                           flat_altitude=0.)
        return env


    def get_some_vns_conf(scenario: 'benchmark.Scenario') -> 'dict':
        conf = {
            'min_time': scenario.time_window_start,
            'max_time': scenario.time_window_end,
            'save_every': 0,
            'save_improvements': False,
            'discrete_elevation_interval': 0,
            'vns': benchmark.vns_configurations['demo']
        }
        conf['vns']['configuration_name'] = 'demo'
        return conf


    # Prepare initial run
    # - VNS params: some_vns_conf
    # - elevation: pl_env
    # - wind: pl_env
    # - Trajectory config: flight_confs_cpp
    # - Firemap: ignitions
    # - Initial plan: Not necessary (c++ plan_vns do it for us)
    scene = benchmark.generate_scenario_singlefire_singleuav_3d()
    pl_env = get_initial_planning_environment(scene)
    some_vns_conf = get_some_vns_conf(scene)
    flight_confs_cpp = [f.as_cpp() for f in scene.flights]
    prop = wildfire.propagate_from_points(pl_env, scene.ignitions,
                                          until=scene.time_window_end + 60 * 10)
    ignitions = prop.ignitions()

    observation_planner = planning.Planner(pl_env, ignitions, scene.flights, some_vns_conf, )
    search_result = observation_planner.compute_plan()

    # TODO: Implement a LSTSsim class that receives a plan and when run:
    # TODO: Obs fire, actual trajectory, plan execution report
    # TODO: The called function must preempt the main loop.
    # neptus.send_plan_to_dune(search_result.final_plan())
    # neptus.get_plan_execution_report() # TODO: not implemented yet.

    # Obtain Obs fire using a firemapper over a fake ground truth.
    # In normal conditions, Obs fire should be provided by neptus
    elevation_draster = pl_env.raster.as_cpp_raster('elevation')
    ignition_draster = ignitions.as_cpp_raster()
    gfmapper = fmapping.GhostFireMapper(
        op.FireData(ignition_draster, elevation_draster))
    # Suposing we follow exactly the planned path
    # executed_path is a tuple with a list of waypoints and a list of the corresponding time
    executed_path = search_result.final_plan().trajectories()[0].sampled_with_time(step_size=10)
    executed_path_wp = executed_path[0]
    executed_path_t = executed_path[1]

    # type: 'op.DRaster'
    obs_fire_raster = gfmapper.observed_firemap(executed_path_wp, executed_path_t,
                                                flight_confs_cpp[0].uav)

    obs_fire = GeoData.from_cpp_raster(obs_fire_raster, 'ignition')

    gdd = GeoDataDisplay.pyplot_figure(obs_fire)
    gdd.add_extension(TrajectoryDisplayExtension, (None,))
    gdd.draw_ignition_contour(geodata=ignitions, cmap=mcm.Greens)
    gdd.draw_ignition_shade()
    plot_plan_trajectories(search_result.final_plan(), gdd)

    print("fin")
    # TODO: Reconstruct fire perimeter from Obs fire

    # search_result = observation_planner.replan_after_time(later)
