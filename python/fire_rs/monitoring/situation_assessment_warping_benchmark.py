"""Benchmark the warping/fusion Situation Assessment algorithm

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
import json

import pytz
import numpy as np
import matplotlib
import matplotlib.cm
import matplotlib.figure
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
from fire_rs.geodata.wildfire import WildfireGraph, warp_firemap, Perimeter
from fire_rs.geodata.wildfire import _warp_image
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
ignition_center = TimedPoint(2776812.5 + 1000, 2212112.5 + 1000, time_start.timestamp())
area_default = ((ignition_center[0] - 2500, ignition_center[0] + 1500),
                (ignition_center[1] - 1500, ignition_center[1] + 2500))


def draw_connecting_arrows(ax, observed_cell_list, corresponding_cells_in_prediction, **kwargs):
    for s, e in zip(observed_cell_list, corresponding_cells_in_prediction):
        # ax.arrow(e[0], e[1], s[0] - e[0], s[1] - e[1], **kwargs)
        ax.annotate("", xy=(s[0], s[1]), xytext=(e[0], e[1]), arrowprops=dict(arrowstyle="->"),
                    **kwargs)


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


def observe_contour(firemap: GeoData, contour: float, n_uavs, uav_speed, duty_cycle=0.25, shift=.0):
    """Extract a contour and then simulate it is observed by N UAVs.
    Each UAV gets an equal size chunk and observed a contigous portion of it (duty_cycle)
    The chunks can be shifted along the perimeter (shift)
    """

    def rotate(l, n):
        return l[n:] + l[:n]

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

    chs = list(chunks(rotate(list(observed_cells.items()), int(shift * len(observed_cells))),
                      int(len(observed_cells) / n_uavs)))
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


def add_custom_legend(gdd: display.GeoDataDisplay, colors, labels, loc="best"):
    colors = colors
    # create a patch (proxy artist) for every color
    patches = [matplotlib.patches.Patch(color=colors[i], label=labels[i]) for i in
               range(len(colors))]
    # put those patched as legend-handles into the legend
    gdd.axes.legend(handles=patches, loc=loc)


def perform_situation_assessment(fire_prop_env: Environment,
                                 observations_dict: ty.Mapping[ty.Tuple[int, int], float],
                                 predicted_firemap: GeoData,
                                 desired_perimeter: float):
    """Find an interpolation function that reconstructs the evolution of the fire
    given current and historic partial information."""
    c_ass_wf = SituationAssessment.WildfireCurrentFusionAssessment(
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


def fig_fire_predicted(corresponding_cells_in_prediction, prediction_propagation,
                       time, observed_cell_list) -> display.GeoDataDisplay:
    gdd = display.GeoDataDisplay.pyplot_figure(prediction_propagation.ignitions(), frame=(0., 0.))
    gdd.draw_ignition_shade(time_range=(0, time), cmap=matplotlib.cm.autumn)
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.colors.ListedColormap("k"),
                              time_range=(0, time))
    # gdd.axes.scatter(
    #     *zip(*(prediction_propagation.prop_data.coordinates(c) for c in
    #            corresponding_cells_in_prediction)), label="predicted (from)", c='C0')
    return gdd


def fig_fire_observed(observed_cell_list,
                      observation_propagation, time) -> display.GeoDataDisplay:
    gdd = display.GeoDataDisplay.pyplot_figure(observation_propagation.ignitions(), frame=(0., 0.))
    gdd.draw_ignition_shade(time_range=(0, time), cmap=matplotlib.cm.autumn)
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.colors.ListedColormap("k"),
                              time_range=(0, time))
    # gdd.axes.scatter(
    #     *zip(*(observation_propagation.prop_data.coordinates(c) for c in observed_cell_list)),
    #     label="observations", c='C1')
    return gdd


def fig_fire_warped(corresponding_cells_in_prediction, observed_cell_list, warped_map,
                    observation_propagation, time,
                    connection_arrows=False) -> display.GeoDataDisplay:
    gdd = display.GeoDataDisplay.pyplot_figure(warped_map, frame=(0., 0.))
    gdd.draw_ignition_shade(time_range=(0, time), cmap=matplotlib.cm.autumn)
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.colors.ListedColormap("k"),
                              time_range=(0, time))

    pred_cells = list((observation_propagation.prop_data.coordinates(c) for c in
                       corresponding_cells_in_prediction))
    obs_cells = list((observation_propagation.prop_data.coordinates(c) for c in
                      observed_cell_list))
    if connection_arrows:
        draw_connecting_arrows(gdd.axes, obs_cells, pred_cells)

    gdd.axes.scatter(*zip(*(observation_propagation.prop_data.coordinates(c) for c in
                            corresponding_cells_in_prediction)), label="predicted (from)",
                     c='C0')
    gdd.axes.scatter(
        *zip(*(observation_propagation.prop_data.coordinates(c) for c in observed_cell_list)),
        label="observed (to)", c='C1')
    gdd.axes.legend(loc='lower left')
    return gdd


def fig_warp_deformation(warped_map, corresponding_cells_in_prediction,
                         observed_cell_list, connection_arrows=False) -> matplotlib.figure.Figure:
    fig = matplotlib.pyplot.figure()
    ax = fig.gca()
    ax.matshow(warped_map["ignition"].T, cmap=matplotlib.cm.inferno)
    ax.scatter(*zip(*corresponding_cells_in_prediction), label="predicted (from)")
    ax.scatter(*zip(*observed_cell_list), label="observed (to)")
    ax.set_xlim((0, warped_map["ignition"].shape[1]))
    ax.set_ylim((0, warped_map["ignition"].shape[0]))
    ax.legend(loc='lower left')
    if connection_arrows:
        draw_connecting_arrows(ax, observed_cell_list, corresponding_cells_in_prediction)
    return fig


def difference_area_perimeters(base_propagation, perimeter1_cells, perimeter2_cells,
                               flood_initial_cell):
    gd1 = np.zeros(base_propagation.prop_data.data.shape)
    gd2 = np.zeros(base_propagation.prop_data.data.shape)
    for cell, time in perimeter1_cells.items():
        gd1.data[cell] = 1
    for cell, time in perimeter2_cells.items():
        gd2.data[cell] = 1
    flood_initial_cell = (
        flood_initial_cell[1], base_propagation.prop_data.data.shape[0] - flood_initial_cell[0])
    surface1 = flood(gd1, flood_initial_cell, connectivity=1)
    surface2 = flood(gd2, flood_initial_cell, connectivity=1)
    area1 = np.count_nonzero(
        surface1) * base_propagation.prop_data.cell_width * base_propagation.prop_data.cell_height
    area2 = np.count_nonzero(
        surface2) * base_propagation.prop_data.cell_width * base_propagation.prop_data.cell_height
    difference_surface = flood(gd1, flood_initial_cell, connectivity=1) ^ flood(gd2,
                                                                                flood_initial_cell,
                                                                                connectivity=1)
    difference_area = np.count_nonzero(
        difference_surface) * base_propagation.prop_data.cell_width * base_propagation.prop_data.cell_height
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


def benchmark(logger, benchmark_id, output_base_folder, current_time, ignition_list,
              prediction_propagation, observation_propagation, observation_passes,
              image_format="pdf", include_start_point_in_assessment=True,
              checker_board_path: ty.Optional[str] = None, observation_sparsity_step=1,
              connection_arrows=False):
    output_path = os.path.join(output_base_folder, benchmark_id)

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    if image_format == 'eps' or image_format == 'pdf':
        matplotlib.rcParams['font.family'] = 'sans-serif'
        matplotlib.rcParams['text.usetex'] = True
    matplotlib.rcParams["axes.autolimit_mode"] = 'round_numbers'

    save_fig_options = {'dpi': 200, 'bbox_inches': 'tight'}
    PROPAGATION_PLOT_NAME = ".".join((benchmark_id + "_predicted_fire", image_format))
    OBSERVED_PLOT_NAME = ".".join((benchmark_id + "_ground_truth", image_format))
    ASSESSMENT_PLOT_NAME = ".".join((benchmark_id + "_assessment_current", image_format))
    OBSERVED_CELLS_PLOT_NAME = ".".join((benchmark_id + "_cells", image_format))
    DEFORMATION_PLOT_NAME = ".".join((benchmark_id + "_deformation", image_format))
    CONTOUR_COMPARISON_PLOT_NAME = ".".join((benchmark_id + "_assessment_contour_comparison",
                                             image_format))
    DIFFERENCE_MAP_PLOT_NAME = ".".join((benchmark_id + "_difference_map", image_format))

    # TODO: print wildfire input data in logs

    w = WildfireGraph(prediction_propagation.prop_data)

    all_perimeter_cells = {}
    all_obs_flattened = {observation_propagation.prop_data.array_index(ignition[0:2]): ignition[2]
                         for
                         ignition in ignition_list} if include_start_point_in_assessment else {}
    obs_flattened_per_observation_pass = []
    outermost_perimeter_cells = {}
    for i, curr_obs_pass in enumerate(observation_passes):
        curr_perimeter_cells, obs_chuncks = observe_contour(
            observation_propagation.ignitions(), curr_obs_pass["time"],
            curr_obs_pass["number_uavs"], 20.0,
            curr_obs_pass["coverage"],
            curr_obs_pass["shift"])
        obs_flattened_per_observation_pass.append(reduce(lambda x, y: {**x, **y}, obs_chuncks))
        all_obs_flattened = {**all_obs_flattened, **obs_flattened_per_observation_pass[-1]}
        outermost_perimeter_cells = curr_perimeter_cells
        all_perimeter_cells = {**all_perimeter_cells, **outermost_perimeter_cells}

    # The cells in the predicted fire map that have the same ignition time as the observations
    observed_cell_list = list(all_obs_flattened.keys())[::observation_sparsity_step]
    corresponding_cells_in_prediction = [
        w.find_parent_or_child_of_time(cell,
                                       observation_propagation.prop_data["ignition"][cell])
        for cell in observed_cell_list]

    # Input information
    logger.info("FIRE ID: {}".format(benchmark_id))
    logger.info("Size: ({:d},{:d})".format(observation_propagation.prop_data.data.shape[0],
                                           observation_propagation.prop_data.data.shape[1]))
    logger.info("Predicted wind ([km/h], [rad]): ({}, {:.2f}×π)".format(
        prediction_propagation.environment.area_wind[0],
        prediction_propagation.environment.area_wind[1] / np.pi))
    logger.info("Real wind ([km/h], [rad]): ({}, {:.2f}×π)".format(
        observation_propagation.environment.area_wind[0],
        observation_propagation.environment.area_wind[1] / np.pi))
    for ignition in ignition_list:
        logger.info("Ignition: ({}, {}) {:%Y-%m-%d %H:%M}".format(
            *observation_propagation.prop_data.array_index(ignition[0:2]),
            datetime.datetime.fromtimestamp(ignition[2])))
    for curr_obs_pass in observation_passes:
        logger.info("Observe contour {:%Y-%m-%d %H:%M}: {:d} UAV, {:.2f}% Coverage".format(
            datetime.datetime.fromtimestamp(curr_obs_pass["time"]),
            curr_obs_pass["number_uavs"], curr_obs_pass["coverage"] * 100))

    gdd_predicted = fig_fire_predicted(corresponding_cells_in_prediction,
                                       prediction_propagation, current_time, observed_cell_list)
    gdd_predicted.figure.savefig(os.path.join(output_path, PROPAGATION_PLOT_NAME),
                                 **save_fig_options)

    gdd_observed = fig_fire_observed(observed_cell_list, observation_propagation, current_time)
    gdd_observed.figure.savefig(os.path.join(output_path, OBSERVED_PLOT_NAME),
                                **save_fig_options)

    # Warping
    warped_map = warp_firemap(prediction_propagation.ignitions(),
                              corresponding_cells_in_prediction,
                              observed_cell_list
                              )

    if checker_board_path:
        import cv2

        checkboard = cv2.imread(checker_board_path, cv2.IMREAD_GRAYSCALE)
        checkerboad_warped = _warp_image(checkboard, corresponding_cells_in_prediction,
                                         observed_cell_list)
        fig = matplotlib.pyplot.figure()
        ax = fig.gca()
        ax.matshow(checkerboad_warped.T, cmap="Greys")
        if connection_arrows:
            draw_connecting_arrows(ax, observed_cell_list, corresponding_cells_in_prediction)
        ax.scatter(*zip(*corresponding_cells_in_prediction), label="from")
        ax.scatter(*zip(*observed_cell_list), label="to")
        ax.set_xlim((0, checkerboad_warped.shape[1]))
        ax.set_ylim((0, checkerboad_warped.shape[0]))
        ax.legend(loc="lower left")
        fig.savefig(os.path.join(output_path, ".".join(("checkerboard_deformation", image_format))),
                    **save_fig_options)
        del fig

    gdd_assessment = fig_fire_warped(corresponding_cells_in_prediction, observed_cell_list,
                                     warped_map, observation_propagation, current_time)
    gdd_assessment.figure.savefig(os.path.join(output_path, ASSESSMENT_PLOT_NAME),
                                  **save_fig_options)

    fig_def = fig_warp_deformation(warped_map, corresponding_cells_in_prediction,
                                   observed_cell_list)
    fig_def.savefig(os.path.join(output_path, DEFORMATION_PLOT_NAME), **save_fig_options)

    # Observed cells figure
    obs_cells_fig = fig_observation_contour(prediction_propagation,
                                            all_obs_flattened,
                                            binary=True)
    obs_cells_fig.draw_ignition_points(
        [prediction_propagation.prop_data.coordinates(c) for c in
         corresponding_cells_in_prediction], color='C0', edgecolor='C0')
    obs_cells_fig.draw_ignition_points(
        [observation_propagation.prop_data.coordinates(c) for c in observed_cell_list],
        color='C1', edgecolor='C1')

    add_custom_legend(obs_cells_fig, ["C0", "C1"], ["predicted (from)", "observed (to)"],
                      loc="lower left")
    if connection_arrows:
        draw_connecting_arrows(
            obs_cells_fig.figure.gca(), [observation_propagation.prop_data.coordinates(c) for c in
                                         observed_cell_list],
            [prediction_propagation.prop_data.coordinates(c) for c in
             corresponding_cells_in_prediction])
    obs_cells_fig.figure.savefig(os.path.join(output_path, OBSERVED_CELLS_PLOT_NAME),
                                 **save_fig_options)

    warped_map_perimeter = Perimeter(warped_map, current_time)
    # FIXME: There is a bug in Perimeter

    diff_map, surface_diff, surface1, surface2 = difference_area_perimeters(
        prediction_propagation, warped_map_perimeter.cells, outermost_perimeter_cells,
        prediction_propagation.prop_data.array_index(ignition_center[0:2]))

    f = matplotlib.pyplot.figure()
    a = f.gca()
    a.matshow(diff_map[..., ::-1].T, cmap=matplotlib.colors.ListedColormap(["w", "k"]))
    f.savefig(os.path.join(output_path, DIFFERENCE_MAP_PLOT_NAME), **save_fig_options)

    obs_uav_cells_fig2 = fig_observation_contour(observation_propagation,
                                                 outermost_perimeter_cells,
                                                 cmap=matplotlib.colors.ListedColormap("C1"),
                                                 binary=True)
    obs_uav_cells_fig2 = fig_observation_contour(observation_propagation,
                                                 warped_map_perimeter.cells,
                                                 cmap=matplotlib.colors.ListedColormap("C2"),
                                                 binary=True,
                                                 gdd=obs_uav_cells_fig2)
    add_custom_legend(obs_uav_cells_fig2, ["C1", "C2"], ["Real", "Estimated"],
                      loc="lower left")
    obs_uav_cells_fig2.figure.savefig(
        os.path.join(output_path, CONTOUR_COMPARISON_PLOT_NAME), **save_fig_options)

    logger.info("SITUATION ASSESSMENT (CURRENT):")
    logger.info("Time: {:%Y-%m-%d %H:%M}".format(datetime.datetime.fromtimestamp(current_time)))
    logger.info("Surface of real fire: {:.3f}km²".format(surface1 / 1000 / 1000))
    logger.info("Surface of the estimated fire: {:.3f}km²".format(surface2 / 1000 / 1000))
    logger.info("Absolute error: {:.3f}km²".format(surface_diff / 1000 / 1000))
    logger.info("Relative error: {:.2f}%".format(surface_diff / surface1 * 100))
    # except Exception as e:
    #     logger.error("Situation Assessment (Current) failed with error: {}".format(str(e)))
    # future_assessment = perform_future_situation_assessment(propagation_env, assessment, current_time+100*60*60)

    print("END")


def prepare_benchmark(duration: float, ignitions, wind_predicted, wind_observed,
                      observation_passes):
    """ roduce the necessary input arguments for a benchmark:

    current_time, ignition_list, prediction_propagation,
         observation_propagation, observation_passes
    """
    current_time = time_start.timestamp() + duration

    ignition_list = [TimedPoint(p["x"], p["y"], time_start.timestamp() + p["time"]) for p in
                     ignitions]

    prediction_propagation = create_wildfire_propagation(
        area_default, ignition_list, wind_speed=wind_predicted[0], wind_direction=wind_predicted[1],
        world=THE_WORLD)
    observation_propagation = create_wildfire_propagation(
        area_default, ignition_list, wind_speed=wind_observed[0], wind_direction=wind_observed[1],
        world=THE_WORLD)

    for op in observation_passes:
        op["time"] += time_start.timestamp()

    return current_time, ignition_list, prediction_propagation, observation_propagation, \
           observation_passes


def main():
    image_format = "svg"
    # checker_board_path = "/home/rbailonr/Downloads/Checkerboard_pattern.svg.png"
    checker_board_path = None
    connection_arrows = True
    include_start_point_in_assessment = True
    observation_sparsity_step = 2

    folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), "output_test",
                          datetime.datetime.now(pytz.timezone('UTC')).isoformat())

    benchmarks = [
        lambda: prepare_benchmark(duration=4 * 60 * 60,
                                  ignitions=[
                                      {"x": ignition_center[0],
                                       "y": ignition_center[1],
                                       "time": 0}
                                  ],
                                  wind_predicted=(6., 0.75 * np.pi),
                                  wind_observed=(5., 0.75 * np.pi),
                                  observation_passes=[
                                      {"time": 4 * 60 * 60,
                                       "number_uavs": 3,
                                       "coverage": 0.33,
                                       "shift": 0.0},
                                  ]),
        lambda: prepare_benchmark(duration=4 * 60 * 60,
                                  ignitions=[
                                      {"x": ignition_center[0],
                                       "y": ignition_center[1],
                                       "time": 0}
                                  ],
                                  wind_predicted=(5., 0.66 * np.pi),
                                  wind_observed=(5., 0.75 * np.pi),
                                  observation_passes=[
                                      {"time": 4 * 60 * 60,
                                       "number_uavs": 3,
                                       "coverage": 0.33,
                                       "shift": -0.2},
                                  ]),
        lambda: prepare_benchmark(duration=4 * 60 * 60,
                                  ignitions=[
                                      {"x": ignition_center[0],
                                       "y": ignition_center[1],
                                       "time": 0}
                                  ],
                                  wind_predicted=(6., 0.66 * np.pi),
                                  wind_observed=(5., 0.75 * np.pi),
                                  observation_passes=[
                                      {"time": 4 * 60 * 60,
                                       "number_uavs": 3,
                                       "coverage": 0.33,
                                       "shift": -0.1},
                                  ]),

        lambda: prepare_benchmark(duration=4 * 60 * 60,
                                  ignitions=[
                                      {"x": ignition_center[0],
                                       "y": ignition_center[1],
                                       "time": 0}
                                  ],
                                  wind_predicted=(6., 0.66 * np.pi),
                                  wind_observed=(5., 0.75 * np.pi),
                                  observation_passes=[
                                      {"time": 4 * 60 * 60,
                                       "number_uavs": 3,
                                       "coverage": 0.33,
                                       "shift": 0.0},
                                  ]),
        lambda: prepare_benchmark(duration=4 * 60 * 60,
                                  ignitions=[
                                      {"x": ignition_center[0] - 750,
                                       "y": ignition_center[1],
                                       "time": 0}
                                  ],
                                  wind_predicted=(6., 0.25 * np.pi),
                                  wind_observed=(4., 0.75 * np.pi),
                                  observation_passes=[
                                      {"time": 4 * 60 * 60,
                                       "number_uavs": 3,
                                       "coverage": 0.75,
                                       "shift": -0.1},
                                  ]),

    ]

    if not os.path.exists(folder):
        os.makedirs(folder)
    logger = create_logger(os.path.join(folder, "output.log"))
    logger.info("Output path: {}".format(folder))

    for i, b in enumerate(benchmarks, start=1):
        benchmark(logger, "assessment_scenario_" + str(i), folder,
                  *b(), image_format=image_format,
                  include_start_point_in_assessment=include_start_point_in_assessment,
                  checker_board_path=checker_board_path,
                  connection_arrows=connection_arrows,
                  observation_sparsity_step=observation_sparsity_step)


if __name__ == "__main__":
    main()
