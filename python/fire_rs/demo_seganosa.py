#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Demo script showcasing propagation and planning in Galicia near SEGANOSA facilities"""

import os
import datetime
import logging

import numpy as np
import matplotlib.cm

import fire_rs.geodata.environment as g_environment
import fire_rs.geodata.display as display

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint
import fire_rs.planning.new_planning as planning
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_plan_trajectories

# Set logger
FORMAT = '%(asctime)-23s %(levelname)-8s [%(name)s]: %(message)s'
logging.basicConfig(format=FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)
logger.setLevel(logging.NOTSET)
planning.up.set_logger(logger)


def plot_sr_with_background(sr: 'planning.up.SearchResult', geodatadisplay, time_range,
                            output_options_plot,
                            plan: 'Union[str, int]' = 'final'):
    """Plot a plan trajectories with background geographic information in a geodata display."""
    # Draw background layers
    for layer in output_options_plot['background']:
        if layer == 'elevation_shade':
            geodatadisplay.draw_elevation_shade(
                with_colorbar=output_options_plot.get('colorbar', True), layer='elevation')
        if layer == 'elevation_planning_shade':
            geodatadisplay.draw_elevation_shade(
                with_colorbar=output_options_plot.get('colorbar', True), layer='elevation_planning')
        elif layer == 'ignition_shade':
            geodatadisplay.draw_ignition_shade(
                with_colorbar=output_options_plot.get('colorbar', True))
        # elif layer == 'observedcells':
        #     geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
        #         planner.expected_observed_map(layer_name="expected_observed"),
        #         layer='expected_observed', color='green', alpha=0.9)
        #     geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
        #         planner.expected_ignited_map(layer_name="expected_ignited"),
        #         layer='expected_ignited', color='red', alpha=0.9)
        elif layer == 'ignition_contour':
            try:
                geodatadisplay.draw_ignition_contour(with_labels=True)
            except ValueError as e:
                logger.exception("ValueError while drawing ignition contour")
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()
        elif layer == 'utilitymap':
            geodatadisplay.TrajectoryDisplayExtension.draw_utility_shade(
                geodata=GeoData.from_cpp_raster(sr.final_plan().utility_map(), 'utility'),
                layer='utility', vmin=0., vmax=1.)

    plot_plan_trajectories(sr.plan(plan), geodatadisplay,
                           layers=output_options_plot["foreground"], time_range=time_range)


if __name__ == "__main__":
    # WOLRD SETTINGS
    FIRERS_DATA_FOLDER = os.environ['FIRERS_DATA']
    FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER,
                                   'dem')
    FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER,
                                    'wind')
    FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER,
                                         'landcover')

    the_world = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                    wind_path=FIRERS_WIND_DATA,
                                    landcover_path=FIRERS_LANDCOVER_DATA)

    seganosa_fire_env = Environment([[2799134.0, 2805134.0], [2296388.0, 2302388.0]],
                                    3, 0, the_world)
    # 6km by 6km around seganosa. Wind: 3m/s W->E

    # FIRE PROPAGATION
    ## Fire started 6 hours ago. Predict until 2 hours in the future
    fire_prop = FirePropagation(seganosa_fire_env)
    four_hours_ago = (datetime.datetime.now() - datetime.timedelta(hours=6)).timestamp()
    now = datetime.datetime.now().timestamp()
    four_hours_from_now = (datetime.datetime.now() + datetime.timedelta(hours=2)).timestamp()
    fire_start = TimedPoint(2802134.0 - 1500.0, 2299388.0, four_hours_ago)
    fire_prop.set_ignition_point(fire_start)

    fire_prop.propagate(until=four_hours_from_now)

    ## Figure terrain + ignition contour + ignition point
    gdd = display.GeoDataDisplay.pyplot_figure(
        seganosa_fire_env.raster.combine(fire_prop.ignitions().slice(["ignition"])),
        frame=(0., 0.))
    gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
    gdd.draw_wind_quiver()
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
    gdd.draw_ignition_points(fire_start)

    # gdd.figure.show()
    gdd.figure.savefig(".".join(("demo_seganosa_propagation", "svg")), dpi=150, bbox_inches='tight')

    # PLANNING
    ## Define the initial plan
    ## Takie off and landing at the same location: maximum flight time 45 minutes

    ## Utility map depen
    f_data = planning.make_fire_data(fire_prop.ignitions(), seganosa_fire_env.raster)
    tw = planning.TimeWindow(0, np.inf)
    utility = planning.make_utility_map(fire_prop.ignitions(), flight_window=tw,
                                        output_layer="utility")
    trajectory_config = [planning.Trajectory(planning.TrajectoryConfig("Trajectory",
                                                                       planning.UAVModels.x8("06"),
                                                                       planning.Waypoint(
                                                                           2801900.0 - 1500,
                                                                           2298900.0 - 1500, 300.0,
                                                                           0.0),
                                                                       planning.Waypoint(
                                                                           2801900.0 - 1500,
                                                                           2298900.0 - 1500, 300.0,
                                                                           0.0),
                                                                       now,
                                                                       2700.0,
                                                                       planning.WindVector(3.0,
                                                                                           0.0)))]
    the_plan = planning.Plan("observation_plan", trajectory_config, f_data, tw,
                             utility.as_cpp_raster("utility"))
    initial_u_map = the_plan.utility_map()

    planner = planning.Planner(the_plan, planning.VNSConfDB.demo_db()["demo"])
    sr_1 = planner.compute_plan(10.0)

    print("Flight mission length: {} minutes".format((sr_1.final_plan().trajectories()[0].start_times[-1] - sr_1.final_plan().trajectories()[0].start_times[0])/60.0))

    ## Display results
    final_u_map = the_plan.utility_map()
    gdd = display.GeoDataDisplay(
        *display.get_pyplot_figure_and_axis(),
        seganosa_fire_env.raster.combine(fire_prop.ignitions()),
        frame=(0., 0.))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
    plot_sr_with_background(sr_1, gdd, (four_hours_ago, four_hours_from_now),
                            {"background": ['elevation_shade', 'ignition_contour', 'wind_quiver'],
                             "foreground": ['trajectory_solid', 'arrows', 'bases']})
    gdd.figure.savefig(".".join(("demo_seganosa_plan", "svg")), dpi=150, bbox_inches='tight')

    gdd = display.GeoDataDisplay(
        *display.get_pyplot_figure_and_axis(),
        seganosa_fire_env.raster.combine(fire_prop.ignitions()),
        frame=(0., 0.))
    gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
    plot_sr_with_background(sr_1, gdd, (four_hours_ago, four_hours_from_now),
                            {"background": ['utilitymap'],
                             "foreground": ['trajectory_solid', 'arrows', 'bases']})
    gdd.figure.savefig(".".join(("demo_seganosa_utility", "svg")), dpi=150, bbox_inches='tight')

    print("end")
