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

import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot

import fire_rs.geodata.display as gdd


class TrajectoryDisplayExtension(gdd.DisplayExtension):
    """Extension to GeoDataDisplay that an observation trajectories."""

    def __init__(self, plan_trajectory):
        self.plan_trajectory = plan_trajectory

    def extend(self, geodatadisplay):
        '''Bounds drawing methods to a GeoDataDisplayInstance.'''
        geodatadisplay.plan_trajectory = self.plan_trajectory
        geodatadisplay.draw_waypoints = types.MethodType(
            TrajectoryDisplayExtension._draw_waypoints_extension, geodatadisplay)
        geodatadisplay.draw_flighttime_path = types.MethodType(
            TrajectoryDisplayExtension._draw_flighttime_path_extension, geodatadisplay)
        geodatadisplay.draw_solid_path = types.MethodType(
            TrajectoryDisplayExtension._draw_solid_path_extension, geodatadisplay)
        geodatadisplay.draw_segments = types.MethodType(
            TrajectoryDisplayExtension._draw_segments_extension, geodatadisplay)
        geodatadisplay.draw_observedcells = types.MethodType(
            TrajectoryDisplayExtension._draw_observedcells, geodatadisplay)

    def _draw_waypoints_extension(self, *args, **kwargs):
        '''Draw path waypoints in a GeoDataDisplay figure.'''
        color = kwargs.get('color', 'C0')
        waypoints = self.plan_trajectory.as_waypoints()
        x = [wp.x for wp in waypoints]
        y = [wp.y for wp in waypoints]
        self._drawings.append(self.axis.scatter(x[::2], y[::2], s=7, c=color, marker='D'))
        self._drawings.append(self.axis.scatter(x[1::2], y[1::2], s=7, c=color, marker='>'))

    def _draw_flighttime_path_extension(self, *args, colorbar_time_range: 'Optional[Tuple[float, float]]' = None,
                                        **kwargs):
        '''Draw trajectory in a GeoDataDisplay figure.

        The color of the trajectory will reflect the time taken by the trajectory.
        Optional argument colorbar_time_range may be a tuple of start and end times in seconds.
        If the Optional argument with_colorbar is set to True, a color bar will be displayed of the trajectory.
        '''
        if len(self.plan_trajectory.segments) < 2:
            return
        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        color_range = np.linspace(self.plan_trajectory.start_time() / 60, self.plan_trajectory.end_time() / 60, len(x))
        color_norm = matplotlib.colors.Normalize(vmin=color_range[0], vmax=color_range[-1])
        if colorbar_time_range is not None:
            color_norm = matplotlib.colors.Normalize(vmin=colorbar_time_range[0]/60, vmax=colorbar_time_range[1]/60)
        self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=color_range,
                                   norm=color_norm, cmap=matplotlib.cm.gist_rainbow,
                                                zorder=TrajectoryDisplayExtension.FOREGROUND_LAYER))
        if kwargs.get('with_colorbar', False):
            cb = self._figure.colorbar(self._drawings[-1], ax=self.axis, shrink=0.65, aspect=20)
            cb.set_label("Flight time [min]")
            self._colorbars.append(cb)

    def _draw_solid_path_extension(self, *args, **kwargs):
        '''Draw trajectory in a GeoDataDisplay figure with solid color

        kwargs:
            color: desired color. Default: C0.
        '''
        if len(self.plan_trajectory.segments) < 2:
            return
        sampled_waypoints = self.plan_trajectory.sampled(step_size=5)
        color = kwargs.get('color', 'C0')
        x = [wp.x for wp in sampled_waypoints]
        y = [wp.y for wp in sampled_waypoints]
        self._drawings.append(self.axis.scatter(x, y, s=1, edgecolors='none', c=color,
                                                zorder=TrajectoryDisplayExtension.FOREGROUND_LAYER))
        # TODO: implement legend

    def _draw_segments_extension(self, *args, **kwargs):
        '''Draw observation segments with start and end points in a GeoDataDisplay figure.'''
        if len(self.plan_trajectory.segments) < 2:
            return
        color = kwargs.get('color', 'C0')
        segments = self.plan_trajectory.segments[1:-1]
        start_x = [s.start.x for s in segments]
        start_y = [s.start.y for s in segments]
        end_x = [s.end.x for s in segments]
        end_y = [s.end.y for s in segments]

        self._drawings.append(self.axis.scatter(start_x, start_y, s=10, edgecolor='black', c=color, marker='D',
                                                zorder=TrajectoryDisplayExtension.FOREGROUND_OVERLAY_LAYER))
        self._drawings.append(self.axis.scatter(end_x, end_y, s=10, edgecolor='black', c=color, marker='>',
                                                zorder=TrajectoryDisplayExtension.FOREGROUND_OVERLAY_LAYER))

        start_base = self.plan_trajectory.segments[0]
        finish_base = self.plan_trajectory.segments[-1]

        self._drawings.append(
            self.axis.scatter(start_base.start.x, start_base.start.y, s=10, edgecolor='black', c=color, marker='o',
                              zorder=TrajectoryDisplayExtension.FOREGROUND_OVERLAY_LAYER))
        self._drawings.append(
            self.axis.scatter(finish_base.start.x, finish_base.start.y, s=10, edgecolor='black', c=color, marker='o',
                              zorder=TrajectoryDisplayExtension.FOREGROUND_OVERLAY_LAYER))

        for i in range(len(segments)):
            self._drawings.append(self.axis.plot([start_x[i], end_x[i]], [start_y[i], end_y[i]], c=color, linewidth=2,
                                                 zorder=TrajectoryDisplayExtension.FOREGROUND_LAYER))

    def _draw_observedcells(self, observations, **kwargs):
        for ptt in observations:
            self.axis.scatter(ptt.as_tuple()[0][0], ptt.as_tuple()[0][1], s=4, c=(0., 1., 0., .5),
                              zorder=TrajectoryDisplayExtension.BACKGROUND_OVERLAY_LAYER, edgecolors='none', marker='s')


def plot_plan_trajectories(plan, geodatadisplay, time_range: 'Optional[Tuple[float, float]]' = None, show=False):
    """Plot the trajectories of a plan."""
    colors = cycle(["red", "green", "blue", "black", "magenta"])
    for traj, color in zip(plan.trajectories(), colors):
        geodatadisplay.plan_trajectory = traj
        geodatadisplay.draw_solid_path(color=color)
        geodatadisplay.draw_segments(color=color)
    if show:
        geodatadisplay.axis.get_figure().show()


def plot_plan_with_background(plan, geodatadisplay, time_range, output_options_plot):
    """Plot a plan trajectories with background geographic information in a geodata display."""
    # Draw background layers
    for layer in output_options_plot['background']:
        if layer == 'elevation_shade':
            geodatadisplay.draw_elevation_shade(
                with_colorbar=output_options_plot.get('colorbar', True))
        elif layer == 'ignition_shade':
            geodatadisplay.draw_ignition_shade(
                with_colorbar=output_options_plot.get('colorbar', True))
        elif layer == 'observedcells':
            geodatadisplay.draw_observedcells(plan.observations())
        elif layer == 'ignition_contour':
            try:
                geodatadisplay.draw_ignition_contour(with_labels=True)
            except ValueError as e:
                logging.warning(e)
        elif layer == 'wind_quiver':
            geodatadisplay.draw_wind_quiver()

    # Plot the plan
    plot_plan_trajectories(plan, geodatadisplay, time_range=time_range, show=True)
