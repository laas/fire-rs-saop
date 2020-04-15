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

import logging
import types

from itertools import cycle
from typing import List, Optional, Sequence, Tuple, Union

import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot
import numpy as np

import fire_rs.geodata.display as gdd
from fire_rs.geodata.geo_data import GeoData
from fire_rs.planning.new_planning import Plan

logger = logging.getLogger(__name__)

TRAJECTORY_COLORS = ["darkgreen", "darkblue", "darkorange", "darkmagenta", "darkred"]


class TrajectoryDisplayExtension(gdd.DisplayExtension):
    """Extension to GeoDataDisplay that an observation trajectories."""

    def __init__(self, base_display: 'gdd.GeoDataDisplayBase', plan_trajectory):
        super().__init__(base_display, self.__class__.__name__)
        self.plan_trajectory = plan_trajectory

    def draw_utility_shade(self, geodata: 'Optional[GeoData]' = None, layer: 'str' = 'utility',
                           with_colorbar: 'bool' = True, label: 'str' = "Utility", **kwargs):
        util_arr = np.array(self._base_display.geodata[layer]) if geodata is None else np.array(
            geodata[layer])

        if 'vmin' not in kwargs:
            kwargs['vmin'] = np.nanmin(util_arr)
        if 'vmax' not in kwargs:
            kwargs['vmin'] = np.nanmax(util_arr)
        if 'cmap' not in kwargs:
            kwargs['cmap'] = matplotlib.cm.Purples_r
        if 'interpolation' not in kwargs:
            kwargs['interpolation'] = 'none'
        if 'extent' not in kwargs:
            kwargs['extent'] = self._base_display.image_scale

        shade = self._base_display.axes.imshow(util_arr.T[::-1, ...], **kwargs)

        self._base_display.drawings.append(shade)
        if with_colorbar:
            self._add_utility_shade_colorbar(shade, label)

    def _add_utility_shade_colorbar(self, shade, label: 'str' = "Utility"):
        cb = self._base_display.figure.colorbar(shade, ax=self._base_display.axes, shrink=0.65,
                                                aspect=20, format="%.1f")
        cb.set_label(label)
        self._base_display.colorbars.append(cb)

    def draw_waypoints(self, *args, **kwargs):
        """Draw path waypoints in a GeoDataDisplay figure."""
        color = kwargs.get('color', 'C0')
        waypoints = self.plan_trajectory.as_waypoints()
        x = [wp.x for wp in waypoints]
        y = [wp.y for wp in waypoints]
        self._base_display.drawings.append(
            self._base_display.axes.scatter(x[::2], y[::2], s=7, c=color, marker='D'))
        self._base_display.drawings.append(
            self._base_display.axes.scatter(x[1::2], y[1::2], s=7, c=color, marker='>'))

    def draw_flighttime_path(self, *args,
                             colorbar_time_range: 'Optional[Tuple[float, float]]' = None,
                             **kwargs):
        """Draw trajectory in a GeoDataDisplay figure.

        The color of the trajectory will reflect the time taken by the trajectory.
        Optional argument colorbar_time_range may be a tuple of start and end times in seconds.
        If the Optional argument with_colorbar is set to True, a color bar will be displayed of the
        trajectory.
        """
        if len(self.plan_trajectory) < 2:
            return

        label = kwargs.get('label', None)

        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        color_range = np.linspace(self.plan_trajectory.start_time() / 60,
                                  self.plan_trajectory.end_time() / 60, len(x))
        color_norm = matplotlib.colors.Normalize(vmin=color_range[0], vmax=color_range[-1])
        if colorbar_time_range is not None:
            color_norm = matplotlib.colors.Normalize(vmin=colorbar_time_range[0] / 60,
                                                     vmax=colorbar_time_range[1] / 60)
        self._base_display.drawings.append(
            self._base_display.axes.scatter(x, y, s=1, edgecolors='none', c=color_range,
                                            label=label, norm=color_norm,
                                            cmap=matplotlib.cm.plasma,
                                            zorder=self._base_display.FOREGROUND_LAYER))
        if kwargs.get('with_colorbar', False):
            cb = self._base_display.figure.colorbar(self._base_display.drawings[-1],
                                                    ax=self._base_display.axes,
                                                    shrink=0.65, aspect=20, format="%d min")
            cb.set_label("Flight time")
            self._base_display.colorbars.append(cb)

    def draw_trajectory_solid(self, trajectory: 'Optional[up.Trajectory]' = None, **kwargs):
        """Draw trajectory in a GeoDataDisplay figure with solid color

        kwargs:
            color: desired color. Default: C0.
        """
        traj = trajectory if trajectory is not None else self.plan_trajectory
        if len(traj) < 2:
            return

        color = kwargs.get('color', 'C0')
        size = kwargs.get('size', 1)
        label = kwargs.get('label', None)
        linestyle = kwargs.get('linestyle', '-')
        time_range = kwargs.get('time_range', (-np.inf, np.inf))

        sampled_waypoints = traj.sampled_with_time(time_range, step_size=5)

        x = [wp.x for i, wp in enumerate(sampled_waypoints[0]) if
             time_range[0] < sampled_waypoints[1][i] < time_range[1]]
        y = [wp.y for i, wp in enumerate(sampled_waypoints[0]) if
             time_range[0] < sampled_waypoints[1][i] < time_range[1]]

        self._base_display.drawings.append(
            self._base_display.axes.plot(x, y, linewidth=size, linestyle=linestyle, c=color,
                                         label=label, zorder=self._base_display.FOREGROUND_LAYER))
        # TODO: implement legend

    def draw_waypoint_trail(self, wp_trail, **kwargs):
        """Draw a waypoint trail in a GeoDataDisplay figure with solid color

        kwargs:
            color: desired color. Default: C0.
        """
        color = kwargs.get('color', 'C0')
        size = kwargs.get('size', 1)
        label = kwargs.get('label', None)
        linestyle = kwargs.get('linestyle', '--')

        self._base_display.drawings.append(
            self._base_display.axes.plot(*(zip(*wp_trail)), linewidth=size, linestyle=linestyle,
                                         c=color, label=label,
                                         zorder=self._base_display.FOREGROUND_LAYER))

    def draw_segments(self, *args, **kwargs):
        """Draw observation segments with start and end points in a GeoDataDisplay figure."""
        if len(self.plan_trajectory) < 2:
            return
        color = kwargs.get('color', 'C0')
        time_range = kwargs.get('time_range', (-np.inf, np.inf))

        py_segments = [s for s in self.plan_trajectory.segments]

        py_modifi_segments = [s for i, s in enumerate(py_segments) if
                              self.plan_trajectory.can_modify(i) and
                              time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]
        py_frozen_segments = [s for i, s in enumerate(py_segments) if
                              not self.plan_trajectory.can_modify(i) and
                              time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]

        # Plot modifiable segments
        start_x_m = [s.start.x for s in py_modifi_segments]
        start_y_m = [s.start.y for s in py_modifi_segments]
        end_x_m = [s.end.x for s in py_modifi_segments]
        end_y_m = [s.end.y for s in py_modifi_segments]

        self._base_display.drawings.append(
            self._base_display.axes.scatter(start_x_m, start_y_m, s=10, edgecolor='black', c=color,
                                            marker='o',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))
        self._base_display.drawings.append(
            self._base_display.axes.scatter(end_x_m, end_y_m, s=10, edgecolor='black', c=color,
                                            marker='>',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        # Plot frozen segments (all but the bases)
        start_x_f = [s.start.x for s in py_frozen_segments if
                     not (s == py_segments[0] or s == py_segments[-1])]
        start_y_f = [s.start.y for s in py_frozen_segments if
                     not (s == py_segments[0] or s == py_segments[-1])]
        end_x_f = [s.end.x for s in py_frozen_segments if
                   not (s == py_segments[0] or s == py_segments[-1])]
        end_y_f = [s.end.y for s in py_frozen_segments if
                   not (s == py_segments[0] or s == py_segments[-1])]

        self._base_display.drawings.append(
            self._base_display.axes.scatter(start_x_f, start_y_f, s=10, edgecolor='black',
                                            c='black', marker='o',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))
        self._base_display.drawings.append(
            self._base_display.axes.scatter(end_x_f, end_y_f, s=10, edgecolor='black', c='black',
                                            marker='>',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        if time_range[0] <= self.plan_trajectory.start_time(0) <= time_range[1]:
            start_base = py_segments[0]
            self._base_display.drawings.append(self._base_display.axes.scatter(
                start_base.start.x, start_base.start.y, s=10, edgecolor=color, c=color, marker='D',
                zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        if time_range[0] <= self.plan_trajectory.start_time(len(self.plan_trajectory) - 1) <= \
                time_range[1]:
            finish_base = py_segments[-1]
            self._base_display.drawings.append(
                self._base_display.axes.scatter(finish_base.start.x, finish_base.start.y, s=10,
                                                edgecolor=color, c=color, marker='D',
                                                zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        start_x = [s.start.x for i, s in enumerate(py_segments) if
                   time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]
        start_y = [s.start.y for i, s in enumerate(py_segments) if
                   time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]
        end_x = [s.end.x for i, s in enumerate(py_segments) if
                 time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]
        end_y = [s.end.y for i, s in enumerate(py_segments) if
                 time_range[0] <= self.plan_trajectory.start_time(i) <= time_range[1]]

        # Draw lines between segment bounds
        for i in range(len(start_x)):
            self._base_display.drawings.append(
                self._base_display.axes.plot([start_x[i], end_x[i]], [start_y[i], end_y[i]],
                                             c=color, linewidth=2,
                                             zorder=self._base_display.FOREGROUND_LAYER))

    def draw_bases(self, trajectory: 'Optional[up.Trajectory]' = None, **kwargs):
        traj = trajectory if trajectory is not None else self.plan_trajectory

        if 's' not in kwargs:
            kwargs['s'] = 10
        if 'color' not in kwargs:
            kwargs['color'] = 'black'
        if 'edgecolor' not in kwargs:
            kwargs['edgecolor'] = 'face'
        if 'marker' not in kwargs:
            kwargs['marker'] = 'D'
        if 'zorder' not in kwargs:
            kwargs['zorder'] = self._base_display.FOREGROUND_OVERLAY_LAYER

        base_s = traj.segments[0]
        base_e = traj.segments[-1]
        if base_s:
            self._base_display.drawings.append(
                self._base_display.axes.scatter(base_s.start.x, base_s.start.y,
                                                **kwargs))
        if base_e:
            self._base_display.drawings.append(
                self._base_display.axes.scatter(base_e.end.x, base_e.end.y, **kwargs))

    def draw_arrows(self, trajectory: 'Optional[up.Trajectory]' = None, **kwargs):
        """Draw trajectory waypoints in a GeoDataDisplay figure."""
        traj = trajectory if trajectory is not None else self.plan_trajectory
        if len(traj) < 2:
            return

        # if 'size' not in kwargs:
        #     kwargs['size'] = 10
        if 'color' not in kwargs:
            kwargs['color'] = 'black'
        if 'edgecolor' not in kwargs:
            kwargs['edgecolor'] = 'face'

        if 'units' not in kwargs:
            kwargs['units'] = 'xy'
        if 'angles' not in kwargs:
            kwargs['angles'] = 'xy'
        if 'scale_units' not in kwargs:
            kwargs['scale_units'] = 'xy'
        if 'scale' not in kwargs:
            kwargs['scale'] = 1

        if 'headaxislength' not in kwargs:
            kwargs['headaxislength'] = 4
        if 'headlength' not in kwargs:
            kwargs['headlength'] = 5
        if 'headwidth' not in kwargs:
            kwargs['headwidth'] = 5
        if 'width' not in kwargs:
            kwargs['width'] = 15
        if 'pivot' not in kwargs:
            kwargs['pivot'] = 'middle'

        if 'zorder' not in kwargs:
            kwargs['zorder'] = self._base_display.FOREGROUND_OVERLAY_LAYER

        py_segments = traj.segments[:]
        py_modifi_segments = [s for i, s in enumerate(py_segments) if
                              traj.can_modify(i)]
        py_frozen_segments = [s for i, s in enumerate(py_segments) if
                              traj.can_modify(i)]

        if py_modifi_segments:
            # Plot modifiable segments
            start_x_m = np.array([s.start.x for s in py_modifi_segments])
            start_y_m = np.array([s.start.y for s in py_modifi_segments])
            start_dir_m = np.array([s.start.dir for s in py_modifi_segments])

            self._base_display.drawings.append(
                self._base_display.axes.quiver(start_x_m, start_y_m,
                                               100 * np.cos(start_dir_m),
                                               100 * np.sin(start_dir_m),
                                               **kwargs))

        # Plot frozen segments (all but the bases)
        # start_x_f = [s.start.x for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        # start_y_f = [s.start.y for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        # start_dir_f = [s.start.dir for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        #
        # for i in range(len(start_x_f)):
        #     self._base_display.drawings.append(
        #         self._base_display.axis.quiver(start_x_f[i], start_y_f[i],
        #                                        100 * np.cos(start_dir_f[i]),
        #                                        100 * np.sin(start_dir_f[i]), units='xy',
        #                                        angles='xy',
        #                                        scale_units='xy', scale=1,
        #                                        facecolor=color, edgecolor='gray',
        #                                        headaxislength=4, headlength=5, headwidth=5,
        #                                        width=15, pivot='middle',
        #                                        zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

    def draw_observedcells(self, observations, **kwargs):
        """Plot observed cells as points"""
        for ptt in observations:
            self._base_display.axes.scatter(ptt[0][0], ptt[0][1], s=4, c=(0., 1., 0., .5),
                                            zorder=self._base_display.BACKGROUND_OVERLAY_LAYER,
                                            edgecolors='none', marker='s')

    def draw_observation_map(self, obs_map: 'Optional[GeoData]' = None, layer='observed',
                             color='green', **kwargs):
        o_map = np.array(obs_map[layer]) if obs_map is not None else np.array(
            self._base_display.geodata[layer])
        o_map[~np.isnan(o_map)] = 1

        # define the colors
        cmap = matplotlib.colors.ListedColormap([color])

        o_map = np.around(o_map.T[::-1, ...] / 60., 1)

        if 'vmin' not in kwargs:
            kwargs['vmin'] = np.nanmin(o_map)
        if 'vmax' not in kwargs:
            kwargs['vmax'] = np.nanmax(o_map)
        if 'interpolation' not in kwargs:
            kwargs['interpolation'] = 'none'
        if 'extent' not in kwargs:
            kwargs['extent'] = self._base_display._image_scale

        shade = self._base_display.axes.imshow(o_map, cmap=cmap, **kwargs)
        self._base_display._drawings.append(shade)


def plot_trajectory(geodatadisplay, trajectory, layers: 'List[str]',
                    time_range: 'Tuple[float, float]' = (-np.inf, np.inf), **kwargs):
    for layer in layers:
        if layer == 'bases':
            geodatadisplay.TrajectoryDisplayExtension.draw_bases(trajectory, **kwargs)
        if layer == 'trajectory_solid':
            geodatadisplay.TrajectoryDisplayExtension.draw_trajectory_solid(
                trajectory, time_range=time_range, **kwargs)
        if layer == 'arrows':
            geodatadisplay.TrajectoryDisplayExtension.draw_arrows(trajectory, **kwargs)


def plot_plan_trajectories(plan, geodatadisplay, layers: 'List[str]',
                           trajectories: 'Union[slice, int]' = slice(None),
                           time_range: 'Tuple[float, float]' = (-np.inf, np.inf),
                           colors: 'Optional[List]' = None, labels: 'Optional[List[str]]' = None):
    """Plot the trajectories of a plan."""
    if not colors:
        colors = TRAJECTORY_COLORS
    colors = cycle(colors)
    if not labels:
        labels = [None, ]
    labels = cycle(labels)

    trajs = plan.trajectories()[trajectories]

    if isinstance(trajectories, int):
        trajs = [trajs]

    for traj, color, label in zip(trajs, colors, labels):
        plot_trajectory(geodatadisplay, traj, layers, time_range,
                        color=color, label=label)


def plot_plan_with_background(plan: Plan, geodatadisplay: gdd, time_range, output_options_plot):
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
            geodatadisplay.draw_ignition_shade(cmap=matplotlib.cm.inferno,
                with_colorbar=output_options_plot.get('colorbar', True))
        elif layer == 'observedcells':
            raise NotImplementedError()
            # geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
            #     planner.expected_observed_map(layer_name="expected_observed"),
            #     layer='expected_observed', color='green', alpha=0.9)
            # geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
            #     planner.expected_ignited_map(layer_name="expected_ignited"),
            #     layer='expected_ignited', color='red', alpha=0.9)
        elif layer == 'ignition_contour':
            try:
                geodatadisplay.draw_ignition_contour(with_labels=True)
            except ValueError as e:
                logger.exception("ValueError while drawing ignition contour")
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()
        elif layer == 'utilitymap':
            geodatadisplay.TrajectoryDisplayExtension.draw_utility_shade(
                geodata=GeoData.from_cpp_raster(plan.utility_map(), 'utility',
                                                projection=geodatadisplay.geodata.projection),
                layer='utility', vmin=0., vmax=1.)

    plot_plan_trajectories(plan, geodatadisplay, layers=output_options_plot["foreground"],
                           time_range=time_range)
