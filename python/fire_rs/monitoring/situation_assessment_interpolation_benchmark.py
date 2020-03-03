"""Benchmark the interpolation Situation Assessment algorithm

Produce Observation, Interpolation and Prediction figures
 """

#  Copyright (c) 2020, CNRS-LAAS
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

#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#
#
import datetime
import logging
import os
import sys
import typing as ty

import pytz
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm
import matplotlib.patches
import skimage.io
import skimage.draw
import skimage.measure
import scipy.interpolate

from fire_rs.geodata.geo_data import GeoData
import fire_rs.geodata.environment as g_environment
import fire_rs.geodata.display as display
import fire_rs.rbf
import fire_rs.geodata.wildfire

import fire_rs.monitoring.supersaop
from fire_rs.monitoring.supersaop import SituationAssessment

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint

from itertools import tee, islice, chain
from functools import reduce

from skimage.segmentation import flood_fill, flood


def previous_and_next(some_iterable):
    prevs, items, nexts = tee(some_iterable, 3)
    prevs = chain([None], prevs)
    nexts = chain(islice(nexts, 1, None), [None])
    return zip(prevs, items, nexts)


################################################################################
# World initialization

FIRERS_DATA_FOLDER = os.environ["FIRERS_DATA"]
FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER, 'dem')
FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER, 'wind')
FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER, 'landcover')

THE_WORLD = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                wind_path=FIRERS_WIND_DATA,
                                landcover_path=FIRERS_LANDCOVER_DATA, wind_mesh_resolution='fine',
                                landcover_to_fuel_remap=g_environment.EVERYTHING_FUELMODEL_REMAP)
THE_WORLD.dem_wind_tile_split = 1
time_start = datetime.datetime(2020, 1, 1, 0, 0, 0, 0)
ignition_center = TimedPoint(2776825.0, 2212175.0, time_start.timestamp())
area_default = ((ignition_center[0] - 2500, ignition_center[0] + 1500),
                (ignition_center[1] - 1500, ignition_center[1] + 2500))


def create_wildfire_propagation(area, ignition: ty.Sequence[TimedPoint], wind_speed=1.,
                                wind_direction=3 * np.pi / 4, world=THE_WORLD) -> FirePropagation:
    """Reference wildfire propagation"""
    fire_env = Environment(area, wind_speed, wind_direction, world)

    fire_prop = FirePropagation(fire_env)
    for ig in ignition:
        fire_prop.set_ignition_point(ig)

    propagation_end_time = np.inf
    fire_prop.propagate(propagation_end_time)

    return fire_prop


def fig_fire_propagation(fire_prop: FirePropagation,
                         original_ignition: ty.Sequence[TimedPoint],
                         max_time: float = np.inf) -> display.GeoDataDisplay:
    """Reference fire display
    Figure terrain + ignition contour + ignition point"""
    gdd = display.GeoDataDisplay.pyplot_figure(
        fire_prop.environment.raster.combine(fire_prop.ignitions().slice(["ignition"])),
        frame=(0., 0.))
    # gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
    # gdd.draw_wind_quiver()
    gdd.draw_ignition_shade(time_range=(0, max_time))
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma, time_range=(0, max_time))
    for o in original_ignition:
        gdd.draw_ignition_points(o)
    return gdd


def observe_contour(firemap: GeoData, contour: float, n_uavs, uav_speed, duty_cycle=0.25):
    """Extract a contour and then simulate it is observed by N UAVs.
    Each UAV gets an equal size chunk and observed a contigous portion of it (duty_cycle)
    The observation time is estimated from the speed
    speed is in m/s"""

    def perimeter_length_cumulated(observed_cells):
        perimeter_l = 0.
        first_c = next(observed_cells.keys().__iter__())
        for prev_c, c, next_c in previous_and_next(observed_cells.keys()):
            if next_c is not None:
                perimeter_l += np.sqrt((next_c[0] - c[0]) ** 2 + (next_c[1] - c[1]) ** 2)
            else:
                perimeter_l += np.sqrt((first_c[0] - c[0]) ** 2 + (first_c[1] - c[1]) ** 2)
        perimeter_l *= firemap.cell_width
        return perimeter_l

    def chunks(lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    # Create contour with scikit-image
    wf_perimeter = fire_rs.geodata.wildfire.Perimeter(firemap, contour)
    observed_cells = wf_perimeter.cells.copy()
    perimeter_length = perimeter_length_cumulated(observed_cells)
    # TODO: Actually changed the observed time according to the UAV speed and stuff
    # Until then, keep the real ignition time
    per_uav_lenght = perimeter_length / n_uavs
    per_uav_observed_lenght = per_uav_lenght * duty_cycle

    chs = list(chunks(list(observed_cells.items()), int(len(observed_cells) / n_uavs)))
    effective_chunks = []
    for n_uav, chunk in zip(range(n_uavs), chs):
        effective_chunks.append(list(chunks(chunk, int(len(chunk) * duty_cycle)))[0])

    effective_chunks_dict = list(map(lambda x: dict(x), effective_chunks))
    return observed_cells, effective_chunks_dict


def fig_observation_contour(base_propagation: FirePropagation,
                            cells: ty.Mapping[ty.Tuple[int, int], float],
                            cmap="Greens_r", binary=True, gdd=None) -> display.GeoDataDisplay:
    gd = GeoData.full_like(base_propagation.prop_data, np.inf)
    for cell, time in cells.items():
        gd.data["ignition"][cell] = time if not binary else 1.0
    if not gdd:
        gdd = display.GeoDataDisplay.pyplot_figure(gd, frame=(0., 0.))
    gdd.draw_ignition_shade(cmap=cmap, geodata=gd)
    return gdd


def add_custom_legend(gdd: display.GeoDataDisplay, colors, labels):
    colors = colors
    # create a patch (proxy artist) for every color
    patches = [matplotlib.patches.Patch(color=colors[i], label=labels[i]) for i in
               range(len(colors))]
    # put those patched as legend-handles into the legend
    gdd.figure.legend(handles=patches)


def perform_situation_assessment(fire_prop_env: Environment,
                                 observations_dict: ty.Mapping[ty.Tuple[int, int], float],
                                 desired_perimeter: float):
    """Find an interpolation function that reconstructs the evolution of the fire
    given current and histroic partial information."""
    c_ass_wf = SituationAssessment.WildfireCurrentAssessment(
        fire_prop_env, observations_dict,
        perimeter_time=datetime.datetime.fromtimestamp(desired_perimeter))
    return c_ass_wf


def perform_future_situation_assessment(fire_prop_env: Environment,
                                        current_assessment: SituationAssessment.WildfireCurrentAssessment,
                                        last_future: float):
    f_ass_wf = fire_rs.monitoring.supersaop.SituationAssessment.WildfireFuturePropagation(
        fire_prop_env, current_assessment.perimeter, {},
        current_assessment.geodata, datetime.datetime.fromtimestamp(last_future))
    return f_ass_wf


def fig_fire_assessment(assessment: SituationAssessment.WildfireCurrentAssessment,
                        original_ignition: ty.Sequence[TimedPoint],
                        max_time: float = np.inf) -> display.GeoDataDisplay:
    """Assessed wildfire map"""
    gdd = display.GeoDataDisplay.pyplot_figure(assessment.geodata, frame=(0., 0.))
    # gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
    # gdd.draw_wind_quiver()
    gdd.draw_ignition_shade(time_range=(0, max_time))
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma, time_range=(0, max_time))
    for o in original_ignition:
        gdd.draw_ignition_points(o)
    return gdd


def difference_area_perimeters(base_propagation, perimeter1_cells, perimeter2_cells,
                               flood_initial_cell):
    gd1 = np.ones(base_propagation.prop_data.data.shape) * 0
    gd2 = np.ones(base_propagation.prop_data.data.shape) * 0
    for cell, time in perimeter1_cells.items():
        gd1.data[cell] = 1
    for cell, time in perimeter2_cells.items():
        gd2.data[cell] = 1

    surface1 = flood(gd1, flood_initial_cell, connectivity=1)
    surface2 = flood(gd2, flood_initial_cell, connectivity=1)
    area1 = np.count_nonzero(
        surface1) * propagation_env.prop_data.cell_width * propagation_env.prop_data.cell_height
    area2 = np.count_nonzero(
        surface2) * propagation_env.prop_data.cell_width * propagation_env.prop_data.cell_height
    difference_surface = flood(gd1, flood_initial_cell, connectivity=1) ^ flood(gd2,
                                                                                flood_initial_cell,
                                                                                connectivity=1)
    difference_area = np.count_nonzero(
        difference_surface) * propagation_env.prop_data.cell_width * propagation_env.prop_data.cell_height
    return difference_surface, difference_area, area1, area2


def create_logger(file_path: ty.Optional[str]):
    _logger = logging.getLogger(__name__)
    _logger.setLevel(logging.DEBUG)

    # Setup logging for this benchmark run
    log_formatter = logging.Formatter(
        '%(asctime)-23s %(levelname)s: %(message)s')

    # Set configuration for the default logger (inherited by others)
    root_logger = logging.getLogger()
    # Print INFO level messages
    log_stdout_hldr = logging.StreamHandler(sys.stdout)
    log_stdout_hldr.setLevel(logging.INFO)
    log_stdout_hldr.setFormatter(log_formatter)
    root_logger.addHandler(log_stdout_hldr)
    # Log everithing to file
    if file_path:
        log_file_hldr = logging.FileHandler(file_path)
        log_file_hldr.setFormatter(log_formatter)
        log_file_hldr.setLevel(logging.INFO)
        root_logger.addHandler(log_file_hldr)

    return _logger


if __name__ == "__main__":
    benchmark_id = run_id = datetime.datetime.now(pytz.timezone('UTC')).isoformat()

    output_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "output", benchmark_id)

    if not os.path.exists(output_path):
        # logger.info("Output path did not exist previoulsy. Making it...")
        os.makedirs(output_path)
    logger = create_logger(os.path.join(output_path, "output.log"))
    logger.info("Output path: {}".format(output_path))

    image_format = "pdf"
    if image_format == 'eps' or image_format == 'pdf':
        matplotlib.rcParams['font.family'] = 'sans-serif'
        matplotlib.rcParams['text.usetex'] = True
    save_fig_options = {'dpi': 200, 'bbox_inches': 'tight'}
    propagation_plot_name = ".".join(("real_fire", image_format))
    assessment_plot_name = ".".join(("assessment_current", image_format))
    observed_plot_name = ".".join(("observed", image_format))
    contours_real_and_assessment = ".".join(("assessment_contour_comparison", image_format))

    CURRENT_TIME = time_start.timestamp() + 15 * 60 * 60

    include_start_point_in_assessment = True

    ignition_list = [
        TimedPoint(ignition_center[0], ignition_center[1], time_start.timestamp()),
        # TimedPoint(ignition_center[0] - 1000, ignition_center[1] + 1000, time_start.timestamp()),
    ]

    propagation_env = create_wildfire_propagation(
        area_default, ignition_list, wind_speed=1., wind_direction=0.65 * np.pi, world=THE_WORLD)
    propagation_plot = fig_fire_propagation(propagation_env, ignition_list,
                                            max_time=CURRENT_TIME)
    propagation_plot.figure.savefig(os.path.join(output_path, propagation_plot_name),
                                    **save_fig_options)

    # Time, n_uavs, coverage
    observation_passes = [
        # {"time": time_start.timestamp() + 3 * 60 * 60, "number_uavs": 3, "coverage": 0.5},
        # {"time": time_start.timestamp() + 5 * 60 * 60, "number_uavs": 3, "coverage": 0.75},
        {"time": time_start.timestamp() + 15 * 60 * 60, "number_uavs": 3, "coverage": 0.5}
    ]
    all_perimeter_cells = {}
    all_obs_flattened = {propagation_env.prop_data.array_index(ignition[0:2]): ignition[2] for
                         ignition in ignition_list} if include_start_point_in_assessment else {}
    obs_flattened_per_observation_pass = []
    outermost_perimeter_cells = {}
    for i, curr_obs_pass in enumerate(observation_passes):
        curr_perimeter_cells, obs_chuncks = observe_contour(
            propagation_env.ignitions(), curr_obs_pass["time"], curr_obs_pass["number_uavs"], 20.0,
            curr_obs_pass["coverage"])
        obs_flattened_per_observation_pass.append(reduce(lambda x, y: {**x, **y}, obs_chuncks))
        all_obs_flattened = {**all_obs_flattened, **obs_flattened_per_observation_pass[-1]}
        outermost_perimeter_cells = curr_perimeter_cells
        all_perimeter_cells = {**all_perimeter_cells, **outermost_perimeter_cells}

    logger.info("FIRE ID: {}".format(benchmark_id))
    logger.info("Size: ({:d},{:d})".format(propagation_env.prop_data.data.shape[0],
                                           propagation_env.prop_data.data.shape[1]))
    for ignition in ignition_list:
        logger.info("Ignition: ({}, {}) {:%Y-%m-%d %H:%M}".format(
            *propagation_env.prop_data.array_index(ignition[0:2]),
            datetime.datetime.fromtimestamp(ignition[2])))
    for curr_obs_pass in observation_passes:
        logger.info("Observe contour {:%Y-%m-%d %H:%M}: {:d} UAV, {:.2f}% Coverage".format(
            datetime.datetime.fromtimestamp(curr_obs_pass["time"]),
            curr_obs_pass["number_uavs"], curr_obs_pass["coverage"] * 100))
    try:
        assessment = perform_situation_assessment(propagation_env.environment, all_obs_flattened,
                                                  CURRENT_TIME)
        assessment_gdd = fig_fire_assessment(assessment, ignition_list, max_time=CURRENT_TIME)
        assessment_gdd.figure.savefig(os.path.join(output_path, assessment_plot_name),
                                      **save_fig_options)

        obs_uav_cells_fig = fig_observation_contour(propagation_env, all_obs_flattened)
        obs_uav_cells_fig.figure.savefig(os.path.join(output_path, observed_plot_name),
                                         **save_fig_options)
        obs_uav_cells_fig2 = fig_observation_contour(propagation_env, outermost_perimeter_cells)
        obs_uav_cells_fig2 = fig_observation_contour(propagation_env, assessment.perimeter.cells,
                                                     cmap="Oranges_r", binary=True,
                                                     gdd=obs_uav_cells_fig2)
        add_custom_legend(obs_uav_cells_fig2, ["g", "orange"], ["test1", "test2"])
        obs_uav_cells_fig2.figure.savefig(
            os.path.join(output_path, contours_real_and_assessment), **save_fig_options)
        matrix, surface_diff, surface1, surface2 = difference_area_perimeters(
            propagation_env, assessment.perimeter.cells, outermost_perimeter_cells,
            propagation_env.prop_data.array_index(ignition_center[0:2]))

        logger.info("SITUATION ASSESSMENT (CURRENT):")
        logger.info("Time: {:%Y-%m-%d %H:%M}".format(datetime.datetime.fromtimestamp(CURRENT_TIME)))
        logger.info("Surface of real fire: {:.3f}km²".format(surface1 / 1000 / 1000))
        logger.info("Surface of the estimated fire: {:.3f}km²".format(surface2 / 1000 / 1000))
        logger.info("Absolute error: {:.3f}km²".format(surface_diff / 1000 / 1000))
        logger.info("Relative error: {:.2f}%".format(surface_diff / surface1 * 100))
    except Exception as e:
        logger.error("Situation Assessment (Current) failed with error: {}".format(str(e)))
    # future_assessment = perform_future_situation_assessment(propagation_env, assessment, CURRENT_TIME+100*60*60)

    print("END")
