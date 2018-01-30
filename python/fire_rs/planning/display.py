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
from fire_rs.planning.planning import Planner


class TrajectoryDisplayExtension(gdd.DisplayExtension):
    """Extension to GeoDataDisplay that an observation trajectories."""

    def __init__(self, base_display: 'gdd.GeoDataDisplayBase', plan_trajectory):
        super().__init__(base_display, self.__class__.__name__)
        self.plan_trajectory = plan_trajectory

    def draw_waypoints(self, *args, **kwargs):
        """Draw path waypoints in a GeoDataDisplay figure."""
        color = kwargs.get('color', 'C0')
        waypoints = self.plan_trajectory.as_waypoints()
        x = [wp.x for wp in waypoints]
        y = [wp.y for wp in waypoints]
        self._base_display.drawings.append(
            self._base_display.axis.scatter(x[::2], y[::2], s=7, c=color, marker='D'))
        self._base_display.drawings.append(
            self._base_display.axis.scatter(x[1::2], y[1::2], s=7, c=color, marker='>'))

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
            self._base_display.axis.scatter(x, y, s=1, edgecolors='none', c=color_range,
                                            label=label, norm=color_norm,
                                            cmap=matplotlib.cm.plasma,
                                            zorder=self._base_display.FOREGROUND_LAYER))
        if kwargs.get('with_colorbar', False):
            cb = self._base_display.figure.colorbar(self._base_display.drawings[-1],
                                                    ax=self._base_display.axis,
                                                    shrink=0.65, aspect=20, format="%d min")
            cb.set_label("Flight time")
            self._base_display.colorbars.append(cb)

    def draw_solid_path(self, *args, **kwargs):
        """Draw trajectory in a GeoDataDisplay figure with solid color

        kwargs:
            color: desired color. Default: C0.
        """
        if len(self.plan_trajectory) < 2:
            return
        color = kwargs.get('color', 'C0')
        size = kwargs.get('size', 1)
        label = kwargs.get('label', None)
        linestyle = kwargs.get('linestyle', '-')
        time_range = kwargs.get('time_range', (-np.inf, np.inf))

        sampled_waypoints = self.plan_trajectory.sampled_with_time(time_range, step_size=5)

        x = [wp.x for i, wp in enumerate(sampled_waypoints[0]) if
             time_range[0] < sampled_waypoints[1][i] < time_range[1]]
        y = [wp.y for i, wp in enumerate(sampled_waypoints[0]) if
             time_range[0] < sampled_waypoints[1][i] < time_range[1]]

        self._base_display.drawings.append(
            self._base_display.axis.plot(x, y, linewidth=size, linestyle=linestyle, c=color,
                                         label=label, zorder=self._base_display.FOREGROUND_LAYER))
        # TODO: implement legend

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
            self._base_display.axis.scatter(start_x_m, start_y_m, s=10, edgecolor='black', c=color,
                                            marker='o',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))
        self._base_display.drawings.append(
            self._base_display.axis.scatter(end_x_m, end_y_m, s=10, edgecolor='black', c=color,
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
            self._base_display.axis.scatter(start_x_f, start_y_f, s=10, edgecolor='black',
                                            c='black', marker='o',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))
        self._base_display.drawings.append(
            self._base_display.axis.scatter(end_x_f, end_y_f, s=10, edgecolor='black', c='black',
                                            marker='>',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        if time_range[0] <= self.plan_trajectory.start_time(0) <= time_range[1]:
            start_base = py_segments[0]
            self._base_display.drawings.append(self._base_display.axis.scatter(
                start_base.start.x, start_base.start.y, s=10, edgecolor=color, c=color, marker='D',
                zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        if time_range[0] <= self.plan_trajectory.start_time(len(self.plan_trajectory) - 1) <= \
                time_range[1]:
            finish_base = py_segments[-1]
            self._base_display.drawings.append(
                self._base_display.axis.scatter(finish_base.start.x, finish_base.start.y, s=10,
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
                self._base_display.axis.plot([start_x[i], end_x[i]], [start_y[i], end_y[i]],
                                             c=color, linewidth=2,
                                             zorder=self._base_display.FOREGROUND_LAYER))

    def draw_arrows(self, *args, **kwargs):
        """Draw observation segments with start and end points in a GeoDataDisplay figure."""
        if len(self.plan_trajectory) < 2:
            return
        color = kwargs.get('color', 'C0')
        size = kwargs.get('size', 2)

        py_segments = self.plan_trajectory.segments[:]
        py_modifi_segments = [s for i, s in enumerate(py_segments) if
                              self.plan_trajectory.can_modify(i)]
        py_frozen_segments = [s for i, s in enumerate(py_segments) if
                              not self.plan_trajectory.can_modify(i)]

        # Plot modifiable segments
        start_x_m = [s.start.x for s in py_modifi_segments]
        start_y_m = [s.start.y for s in py_modifi_segments]
        end_x_m = [s.end.x for s in py_modifi_segments]
        end_y_m = [s.end.y for s in py_modifi_segments]

        for i in range(len(start_x_m)):
            self._base_display.drawings.append(
                self._base_display.axis.arrow(start_x_m[i], start_y_m[i],
                                              end_x_m[i] - start_x_m[i],
                                              end_y_m[i] - start_y_m[i],
                                              facecolor=color, edgecolor='black',
                                              length_includes_head=True,
                                              width=size,
                                              zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        # Plot frozen segments (all but the bases)
        start_x_f = [s.start.x for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        start_y_f = [s.start.y for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        end_x_f = [s.end.x for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]
        end_y_f = [s.end.y for s in py_frozen_segments[1:len(py_frozen_segments) - 1]]

        for i in range(len(start_x_f)):
            self._base_display.drawings.append(
                self._base_display.axis.arrow(start_x_f[i], start_y_f[i],
                                              end_x_f[i] - start_x_f[i],
                                              end_y_f[i] - start_y_f[i],
                                              facecolor=color, edgecolor='black',
                                              length_includes_head=True,
                                              width=size,
                                              zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

        start_base = py_segments[0]
        finish_base = py_segments[-1]

        self._base_display.drawings.append(self._base_display.axis.scatter(
            start_base.start.x, start_base.start.y, s=10, edgecolor=color, c=color, marker='D',
            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))
        self._base_display.drawings.append(
            self._base_display.axis.scatter(finish_base.start.x, finish_base.start.y, s=10,
                                            edgecolor=color, c=color, marker='D',
                                            zorder=self._base_display.FOREGROUND_OVERLAY_LAYER))

    def draw_observedcells(self, observations, **kwargs):
        """Plot observed cells as points"""
        for ptt in observations:
            self._base_display.axis.scatter(ptt[0][0], ptt[0][1], s=4, c=(0., 1., 0., .5),
                                            zorder=self._base_display.BACKGROUND_OVERLAY_LAYER,
                                            edgecolors='none', marker='s')

    def draw_observation_map(self, obs_map: 'Optional[GeoData]' = None, layer='observed',
                             color='green', **kwargs):
        o_map = np.array(obs_map[layer]) if obs_map is not None else np.array(
            self._base_display.geodata[layer])
        o_map[~np.isnan(o_map)] = 1

        # define the colors
        cmap = matplotlib.colors.ListedColormap([color])

        shade = gdd.plot_ignition_shade(self._base_display.axis, self._base_display.x_mesh,
                                        self._base_display.y_mesh,
                                        np.around(o_map.T[::-1, ...] / 60., 1),
                                        dx=self._base_display.geodata.cell_width,
                                        dy=self._base_display.geodata.cell_height,
                                        image_scale=self._base_display.image_scale, cmap=cmap,
                                        **kwargs)
        self._base_display.drawings.append(shade)


def plot_plan_trajectories(plan, geodatadisplay,
                           trajectories: 'Union[slice, int]' = slice(None),
                           time_range: 'Tuple[float, float]' = (-np.inf, np.inf),
                           colors: 'Optional[List]' = None, sizes: 'Optional[List[int]]' = None,
                           labels: 'Optional[List[str]]' = None,
                           linestyles: 'Optional[List[str]]' = None,
                           draw_segments: bool = True, draw_path: bool = True,
                           draw_flighttime_path: bool = False, show=False):
    """Plot the trajectories of a plan."""
    if not colors:
        colors = ["darkred", "darkgreen", "darkblue", "darkorange", "darkmagenta"]
    colors = cycle(colors)
    if not sizes:
        sizes = [1, ]
    sizes = cycle(sizes)
    if not labels:
        labels = [None, ]
    labels = cycle(labels)
    if not linestyles:
        linestyles = ['-', ]
    linestyles = cycle(linestyles)

    trajs = plan.trajectories()[trajectories]

    if isinstance(trajectories, int):
        trajs = [trajs]

    for traj, color, size, label, linestyle in zip(trajs, colors, sizes, labels, linestyles):
        geodatadisplay.TrajectoryDisplayExtension.plan_trajectory = traj
        if draw_path:
            geodatadisplay.TrajectoryDisplayExtension.draw_solid_path(time_range=time_range,
                                                                      color=color, size=size,
                                                                      label=label,
                                                                      linestyle=linestyle)
        if draw_flighttime_path:
            logging.warning("time_Range not implemented for this method")
            geodatadisplay.TrajectoryDisplayExtension.draw_flighttime_path(with_colorbar=True)

        if draw_segments:
            geodatadisplay.TrajectoryDisplayExtension.draw_segments(time_range=time_range,
                                                                    color=color,
                                                                    size=size * 5)
    if show:
        geodatadisplay.figure.show()


def plot_plan_with_background(planner: 'Planner', geodatadisplay, time_range, output_options_plot,
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
        elif layer == 'observedcells':
            geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
                planner.expected_observed_map(layer_name="expected_observed"),
                layer='expected_observed', color='green', alpha=0.9)
            geodatadisplay.TrajectoryDisplayExtension.draw_observation_map(
                planner.expected_ignited_map(layer_name="expected_ignited"),
                layer='expected_ignited', color='red', alpha=0.9)
        elif layer == 'ignition_contour':
            try:
                geodatadisplay.draw_ignition_contour(with_labels=True)
            except ValueError as e:
                logging.warning(e)
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()

    # Plot the plan
    if plan == 'final':
        plot_plan_trajectories(planner.search_result.final_plan(), geodatadisplay,
                               time_range=time_range, show=True)
    elif plan == 'initial':
        plot_plan_trajectories(planner.search_result.initial_plan(), geodatadisplay,
                               time_range=time_range, show=True)
    else:
        plot_plan_trajectories(planner.search_result.intermediate_plans[plan], geodatadisplay,
                               time_range=time_range, show=True)
