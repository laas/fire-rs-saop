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

"""Display GeoData information on matplotlib figures"""

__all__ = ['GeoDataDisplay', 'GeoDataDisplayBase', 'DisplayExtension', 'UAVDisplayExtension']

from collections.abc import Sequence
import datetime
from typing import Optional, Tuple, Type, Union, Dict

import matplotlib
import matplotlib.axis
import matplotlib.cm
import matplotlib.dates
import matplotlib.figure
import matplotlib.ticker
import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms
import numpy as np

from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.path import Path

from fire_rs.geodata.geo_data import GeoData
from fire_rs.deprecation import deprecated

house_marker = Path(np.array([[0., 0.],
                              [1., 0.],
                              [1., 1.],
                              [2., 1.],
                              [2., 0.],
                              [3., 0.],
                              [3., 2.],
                              [1.5, 3.],
                              [0., 2.],
                              [0., 0.]]) - np.array([1.5, 1.5]), closed=True)


class EngOffsetFormatter(matplotlib.ticker.EngFormatter):

    def __init__(self, unit="", places=None, offset=0):
        super().__init__(unit, places)
        self._offset = offset

    def __call__(self, x, pos=None):
        s = "%s%s" % (self.format_eng(x + self._offset), self.unit)
        return self.fix_minus(s)


def get_pyplot_figure_and_axis():
    fire_fig = plt.figure()
    fire_ax = fire_fig.gca(aspect='equal', xlabel="East", ylabel="North")

    ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
    fire_ax.yaxis.set_major_formatter(ax_formatter)
    fire_ax.xaxis.set_major_formatter(ax_formatter)
    return fire_fig, fire_ax


def plot_uav(ax, position: Tuple[float, float], orientation: float, size=1, **kwargs):
    if 'facecolor' not in kwargs:
        kwargs['facecolor'] = 'blue'
    if 'edgecolor' not in kwargs:
        kwargs['edgecolor'] = 'black'

    plane_vertices = (np.array([[3.5, 6], [4, 5], [4, 4], [7, 4], [7, 3],
                                [4, 3], [4, 1], [5, 1], [5, 0], [2, 0],
                                [2, 1], [3, 1], [3, 3], [0, 3], [0, 4],
                                [3, 4], [3, 5]]) - np.array([3.5, 3])) * size
    r = np.array([[np.cos(orientation), -np.sin(orientation)],
                  [np.sin(orientation), np.cos(orientation)]])

    plane_polygon = matplotlib.patches.Polygon(np.matmul(plane_vertices, r) + position,
                                               closed=True, fill=True, **kwargs)
    return ax.add_patch(plane_polygon)


@deprecated
def plot_uav_deprecated(ax, uav_state, size=1, facecolor='blue', edgecolor='black', **kwargs):
    plane_vertices = (np.array([[3.5, 6], [4, 5], [4, 4], [7, 4], [7, 3],
                                [4, 3], [4, 1], [5, 1], [5, 0], [2, 0],
                                [2, 1], [3, 1], [3, 3], [0, 3], [0, 4],
                                [3, 4], [3, 5]]) - np.array([3.5, 3])) * size
    r = np.array([[np.cos(-uav_state[2] + np.pi / 2), -np.sin(-uav_state[2] + np.pi / 2)],
                  [np.sin(-uav_state[2] + np.pi / 2), np.cos(-uav_state[2] + np.pi / 2)]])

    plane_polygon = matplotlib.patches.Polygon(np.matmul(plane_vertices, r) + uav_state[0:2],
                                               closed=True, fill=True,
                                               facecolor=facecolor, edgecolor=edgecolor, **kwargs)
    return ax.add_patch(plane_polygon)


@deprecated
def plot_elevation_contour(ax, x, y, z, **kwargs):
    contour = ax.contour(x, y, z, 15, cmap=kwargs.get('cmap', matplotlib.cm.gist_earth))
    # labels = plt.clabel(contour, inline=1, fontsize=10)
    return contour


@deprecated
def plot_wind_flow(ax, x, y, wx, wy, wvel, **kwargs):
    return ax.streamplot(x, y, wx, wy, density=1, linewidth=1, color='dimgrey')


@deprecated
def plot_wind_quiver(ax, x, y, wx, wy, **kwargs):
    return ax.quiver(x, y, wx, wy, pivot=kwargs.get('pivot', 'middle'),
                     color=kwargs.get('color', 'dimgrey'), **kwargs)


class GeoDataDisplayBase:
    BACKGROUND_LAYER = 0
    BACKGROUND_OVERLAY_LAYER = 100
    FOREGROUND_LAYER = 200
    FOREGROUND_OVERLAY_LAYER = 300

    def __init__(self, figure: 'matplotlib.figure.Figure', axes: 'matplotlib.axes.Axes',
                 geodata: 'GeoData', frame: 'Optional[Tuple[float, float]]' = None):
        """
        Initialize GeoDataDisplayBase
        :param figure: a matplotlib figure
        :param axes: a matplotlib axis
        :param geodata: a geodata
        :param frame: an optional reference frame for the figure data.
        ie. (0., 0.) for X and Y with local reference instead of Lambert93 by default.
        """
        self._figure, self._axes = figure, axes
        self._geodata = geodata

        if frame is None:
            self._frame = (
                geodata.x_offset + geodata.cell_width/2, geodata.y_offset + geodata.cell_width/2)
        else:
            self._frame = frame

        x = np.arange(geodata.max_x)
        self._x_ticks = (x * geodata.cell_width) + geodata.x_offset + geodata.cell_width/2
        y = np.arange(geodata.max_y)
        self._y_ticks = (y * geodata.cell_height) + geodata.y_offset + geodata.cell_width/2

        x_fmtr = EngOffsetFormatter(unit='m',
                                    offset=-geodata.x_offset - geodata.cell_width/2 + self._frame[0])
        y_fmtr = EngOffsetFormatter(unit='m',
                                    offset=-geodata.y_offset - geodata.cell_width/2 + self._frame[1])
        self._axes.xaxis.set_major_formatter(x_fmtr)
        self._axes.yaxis.set_major_formatter(y_fmtr)

        self._image_scale = (
            self._x_ticks[0], self._x_ticks[-1], self._y_ticks[0], self._y_ticks[-1])

        self._x_mesh, self._y_mesh = np.meshgrid(x, y)

        self._drawings = []
        self._colorbars = []

        self._extensions = {}

    @property
    def drawings(self):
        return self._drawings

    @property
    def colorbars(self):
        return self._colorbars

    @property
    def figure(self):
        return self._figure

    @property
    def axes(self):
        return self._axes

    @property
    def x_ticks(self):
        return self._x_ticks

    @property
    def y_ticks(self):
        return self._y_ticks

    @property
    def x_mesh(self):
        return self._x_mesh

    @property
    def y_mesh(self):
        return self._y_mesh

    @property
    def image_scale(self):
        return self._image_scale

    @property
    def geodata(self):
        return self._geodata

    @classmethod
    def pyplot_figure(cls, geodata, frame=None):
        figure, axis = get_pyplot_figure_and_axis()
        return cls(figure, axis, geodata, frame=frame)

    def close(self):
        plt.close(self.figure)

    def clear_axis(self):
        self._axes.cla()
        # drawings and colorbar references must be cleared to prevent increasing memory usage
        self._drawings = []
        self._colorbars = []
        self._axes.set_aspect('equal')
        self._axes.set_xlabel("East")
        self._axes.set_ylabel("North")
        x_fmtr = EngOffsetFormatter(
            unit='m', offset=-self._geodata.x_offset - self._geodata.cell_width/2 + self._frame[0])
        y_fmtr = EngOffsetFormatter(
            unit='m', offset=-self._geodata.y_offset - self._geodata.cell_width/2 + self._frame[1])
        self._axes.xaxis.set_major_formatter(x_fmtr)
        self._axes.yaxis.set_major_formatter(y_fmtr)

    def add_extension(self, extensionclass: 'Type[DisplayExtension]', extension_args: 'tuple' = (),
                      extension_kwargs: 'dict' = {}):
        ext = extensionclass(self, *extension_args, **extension_kwargs)
        self._extensions[ext.name] = ext

    def remove_extension(self, ext_name: 'str'):
        self._extensions.pop(ext_name)  # Remove "extension_name" element from dict

    def __getattr__(self, ext_name: 'str'):
        return self._extensions[ext_name]

    def legend(self):
        """Draw legend of labeled plots"""
        self._axes.legend()


class MinuteDateFormatter(matplotlib.dates.DateFormatter):
    """Format date from a timestamp in minutes"""

    def __call__(self, x, pos=0):
        # TZ should not be used, because ROS only work on localtime
        # without any time zone consideration
        return datetime.datetime.fromtimestamp(x * 60, None).strftime(self.fmt)


class SecondDateFormatter(matplotlib.dates.DateFormatter):
    """Format date from a timestamp in seconds"""

    def __call__(self, x, pos=0):
        # TZ should not be used, because ROS only work on localtime
        # without any time zone consideration
        # WARNING: If the fire start is not dated,
        # the displayed hour will be off by the current timezone
        # timestamp "0.000000" is shown 1:00 in France (2:00 in summer)
        return datetime.datetime.fromtimestamp(x, None).strftime(self.fmt)


class GeoDataDisplay(GeoDataDisplayBase):
    """Draw GeoData information on a matplotlib figure.
    
    'draw_' methods should be called in the order from background to foreground.
    """

    def draw_elevation_shade(self, geodata: 'Optional[GeoData]' = None, layer: 'str' = 'elevation',
                             with_colorbar: 'bool' = False, label: 'str' = "Elevation", **kwargs):
        gd = self._geodata if geodata is None else geodata

        z = gd[layer].T[::-1, ...]

        if 'vmin' not in kwargs:
            kwargs['vmin'] = np.nanmin(z)
        if 'vmax' not in kwargs:
            kwargs['vmax'] = np.nanmax(z)
        if 'cmap' not in kwargs:
            kwargs['cmap'] = matplotlib.cm.gist_earth
        if 'interpolation' not in kwargs:
            kwargs['interpolation'] = 'none'
        if 'extent' not in kwargs:
            kwargs['extent'] = self._image_scale

        ls = LightSource(azdeg=315, altdeg=35)
        axim = self.axes.imshow(
            ls.shade(z, cmap=kwargs['cmap'], blend_mode='soft', vert_exag=1, dx=gd.cell_width,
                     dy=gd.cell_height, vmin=kwargs['vmin'], vmax=kwargs['vmax']), **kwargs)

        self._drawings.append(axim)
        if with_colorbar:
            self._add_elevation_shade_colorbar(axim, label)

        return axim

    def _add_elevation_shade_colorbar(self, axim, label):
        cb = self._figure.colorbar(axim, ax=self.axes, shrink=0.65, aspect=20, format="%d m")
        cb.set_label(label)
        self._colorbars.append(cb)

    def draw_wind_quiver(self, geodata: 'Optional[GeoData]' = None,
                         layer: 'Tuple[str, str]' = ('wind_velocity', 'wind_angle'),
                         skip: 'int' = 10, **kwargs):
        gd = self._geodata if geodata is None else geodata

        wind_vel = gd[layer[0]][::skip, ::skip]
        wind_ang = gd[layer[1]][::skip, ::skip]
        wx = wind_vel * np.cos(wind_ang)
        wy = wind_vel * np.sin(wind_ang)

        if 'pivot' not in kwargs:
            kwargs['pivot'] = 'middle'
        if 'color' not in kwargs:
            kwargs['color'] = 'dimgrey'

        w_quiver = self.axes.quiver(*np.meshgrid(self._x_ticks[::skip], self._y_ticks[::skip]),
                                    wx, wy, **kwargs)
        self.drawings.append(w_quiver)

        return w_quiver

    def draw_ignition_contour(self, geodata: 'Optional[GeoData]' = None, layer: 'str' = 'ignition',
                              time_range: 'Optional[Tuple[float, float]]' = None,
                              with_labels: 'bool' = False, n_fronts=None, **kwargs):
        gd = self._geodata if geodata is None else geodata

        igni = np.array(gd[layer])
        igni[igni >= np.inf] = np.nan  # mask non ignited cells

        if time_range:
            igni[igni > time_range[1]] = time_range[1]
            igni[igni < time_range[0]] = time_range[0]

        igni = igni.T

        # Determine how many contour lines we are going to draw
        lim = (np.nanmin(igni), np.nanmax(igni))
        igni[np.isnan(igni)] = np.inf
        igni[igni > lim[1]] = lim[1]
        igni[igni < lim[0]] = lim[0]
        nfronts = int(np.clip(int((lim[1] - lim[0]) / 3600) * 10, 3, 10)) if n_fronts is None \
            else n_fronts

        if 'cmap' not in kwargs:
            kwargs['cmap'] = matplotlib.cm.YlOrRd

        # contour(X, Y, Z, N, **kwargs)
        firecont = self.axes.contour(self._x_ticks, self._y_ticks, igni, nfronts, **kwargs)

        if with_labels:
            self._add_ignition_contour_labels(firecont)

        return firecont

    def _add_ignition_contour_labels(self, firecont):
        # self.axes.clabel(firecont, inline=True, inline_spacing=1, linewidth=2, fontsize='smaller',
        #                  fmt='%.0f')
        self.axes.clabel(firecont, inline=True, inline_spacing=1, fontsize='smaller',
                         fmt=SecondDateFormatter('%H:%M'))

    def draw_ignition_shade(self, geodata: 'Optional[GeoData]' = None, layer: 'str' = 'ignition',
                            time_range: 'Optional[Tuple[float, float]]' = None,
                            with_colorbar: 'bool' = False, label: 'str' = "Ignition time",
                            **kwargs):
        gd = self._geodata if geodata is None else geodata

        igni = np.array(gd[layer])
        igni[igni >= np.inf] = np.nan  # mask non ignited cells

        if time_range:
            igni[igni > time_range[1]] = np.nan
            igni[igni < time_range[0]] = np.nan

        igni = np.around(igni.T[::-1, ...], 0)  # Convert and clip to exact seconds

        if 'vmin' not in kwargs:
            kwargs['vmin'] = np.nanmin(igni)
        else:
            kwargs['vmin'] /= 60.
        if 'vmax' not in kwargs:
            kwargs['vmax'] = np.nanmax(igni)
        else:
            kwargs['vmax'] /= 60.
        if 'cmap' not in kwargs:
            kwargs['cmap'] = matplotlib.cm.YlOrRd
        if 'interpolation' not in kwargs:
            kwargs['interpolation'] = 'none'
        if 'extent' not in kwargs:
            kwargs['extent'] = self._image_scale

        shade = self.axes.imshow(igni, **kwargs)
        self._drawings.append(shade)

        if with_colorbar:
            self._add_ignition_shade_colorbar(shade, label)

        return shade

    def _add_ignition_shade_colorbar(self, shade, label: 'str'):
        cb = self._figure.colorbar(shade, ax=self.axes, shrink=0.65, aspect=20,
                                   format=SecondDateFormatter('%d/%m/%y %H:%M'))
        cb.set_label(label)
        self._colorbars.append(cb)

    def draw_base(self, bases: 'Union[[(float, float)], (float, float)]', **kwargs):
        """Draw a point marked as bases in a GeoDataDisplay figure."""
        if 's' not in kwargs:
            kwargs['s'] = 200
        if 'edgecolor' not in kwargs:
            kwargs['edgecolor'] = 'black'

        return self._draw_scatter(bases, marker=house_marker, **kwargs)

    def draw_base_tagged(self, bases: 'Dict[str, Tuple[float, float]]', **kwargs):
        """Draw a point marked as bases in a GeoDataDisplay figure."""
        if 's' not in kwargs:
            kwargs['s'] = 200
        if 'edgecolor' not in kwargs:
            kwargs['edgecolor'] = 'black'

        return self._draw_scatter(bases, marker=house_marker, **kwargs)

    def draw_ignition_points(self, ignition_points: 'Union[[(float, float)], (float, float)]',
                             **kwargs):
        """Draw one or multiple ignition points in a GeoDataDisplay figure."""
        if 'color' not in kwargs:
            kwargs['color'] = 'red'
        if 'edgecolor' not in kwargs:
            kwargs['edgecolor'] = 'black'
        if 'marker' not in kwargs:
            kwargs['marker'] = 'o'
        kwargs["zorder"] = GeoDataDisplayBase.FOREGROUND_OVERLAY_LAYER

        return self._draw_scatter(ignition_points, **kwargs)

    def _draw_scatter(self, points: 'Union[[(float, float)], (float, float)]', **kwargs):
        """Draw one or multiple points in a GeoDataDisplay figure."""
        ip_arr = np.array(points)

        scattered = self.axes.scatter(ip_arr[..., 0], ip_arr[..., 1], **kwargs)
        self._drawings.append(scattered)

        return scattered

    def draw_uav(self, position: Tuple[float, float], orientation: float, size=25, **kwargs):
        """Draw a uav symbol in a GeoDataDisplay figure."""
        if "zorder" not in kwargs:
            kwargs["zorder"] = GeoDataDisplayBase.FOREGROUND_OVERLAY_LAYER
        self._drawings.append(plot_uav(self.axes, position, orientation, size, **kwargs))


class DisplayExtension:

    def __init__(self, base_display: 'GeoDataDisplayBase', name: 'str'):
        self._name = name  # type: 'str'
        self._base_display = base_display  # type: 'GeoDataDisplayBase'

    @property
    def name(self) -> 'str':
        return self._name


@deprecated("Use basic GeoDataDisplay instead")
class UAVDisplayExtension(DisplayExtension):
    """Extension to GeoDataDisplay that draws small uavs"""

    def __init__(self, base_display: 'GeoDataDisplayBase', uav_state_list):
        super().__init__(base_display, self.__class__.__name__)
        self.uav_state_list = uav_state_list

    @deprecated("Use draw_uav of GeoDataDisplay instead")
    def draw_uav(self, *args, **kwargs):
        """Draw ignition point in a GeoDataDisplay figure."""
        for p in self.uav_state_list:
            self._base_display.drawings.append(
                plot_uav_deprecated(self._base_display.axes, p, *args, **kwargs))


@deprecated
def plot3d_elevation_shade(ax, x, y, z, dx=25, dy=25, **kwargs):
    ls = LightSource(azdeg=120, altdeg=45)
    rgb = ls.shade(z, cmap=matplotlib.cm.terrain, vert_exag=0.1, blend_mode='overlay')
    return ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5, linewidth=0,
                           antialiased=True, shade=True)


@deprecated
def plot3d_wind_arrows(ax, x, y, z, wx, wy, wz, **kwargs):
    return ax.quiver(x, y, z, wx, wy, wz, pivot='middle', cmap=matplotlib.cm.viridis)
