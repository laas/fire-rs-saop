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

__all__ = ['GeoDataDisplay', 'IgnitionPointDisplayExtension', 'UAVDisplayExtension']

import abc
import types

import matplotlib
import matplotlib.cm
import matplotlib.figure
import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms
import numpy as np

from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from mpl_toolkits.mplot3d import Axes3D


def get_pyplot_figure_and_axis():
    fire_fig = plt.figure()
    fire_ax = fire_fig.gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

    ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
    fire_ax.yaxis.set_major_formatter(ax_formatter)
    fire_ax.xaxis.set_major_formatter(ax_formatter)
    return fire_fig, fire_ax


def plot_uav(ax, uav_state, size=1, facecolor='blue', edgecolor='black', **kwargs):
    plane_vertices = (np.array([[3.5, 6], [4, 5], [4, 4], [7, 4], [7, 3],
                      [4,   3], [4, 1], [5, 1], [5, 0], [2, 0],
                      [2, 1], [3, 1], [3, 3], [0, 3], [0, 4],
                      [3, 4], [3, 5]]) - np.array([3.5, 3])) * size
    r = np.array([[np.cos(-uav_state[2]+np.pi/2), -np.sin(-uav_state[2]+np.pi/2)],
                  [np.sin(-uav_state[2]+np.pi/2), np.cos(-uav_state[2]+np.pi/2)]])

    plane_polygon = matplotlib.patches.Polygon(np.matmul(plane_vertices, r) + uav_state[0:2], closed=True, fill=True,
                                               facecolor=facecolor, edgecolor=edgecolor, **kwargs)
    return ax.add_patch(plane_polygon)


def plot_ignition_shade(ax, x, y, ignition_times, dx=25, dy=25, image_scale=None, **kwargs):
    cbar_lim = (np.nanmin(ignition_times), np.nanmax(ignition_times))
    if not image_scale:
        image_scale = (x[0][0], x[0][x.shape[0] - 1], y[0][0], y[y.shape[0] - 1][0])
    return ax.imshow(ignition_times, extent=image_scale, vmin=cbar_lim[0], vmax=cbar_lim[1],
                     cmap=matplotlib.cm.gist_heat, **kwargs)


def plot_ignition_contour(ax, x, y, ignition_times, nfronts=None, **kwargs):
    if not nfronts:
        lim = (np.nanmin(ignition_times), np.nanmax(ignition_times))
        nfronts = int(np.clip(int((lim[1]-lim[0])/60)*10, 3, 20))
    return ax.contour(x, y, ignition_times, nfronts, cmap=matplotlib.cm.gist_rainbow, **kwargs)


def plot_elevation_contour(ax, x, y, z, **kwargs):
    contour = ax.contour(x, y, z, 15, cmap=matplotlib.cm.gist_earth)
    # labels = plt.clabel(contour, inline=1, fontsize=10)
    return contour


def plot_elevation_shade(ax, x, y, z, dx=25, dy=25, image_scale=None, **kwargs):
    cbar_lim = (np.nanmin(z), np.nanmax(z))
    if not image_scale:
        image_scale = (x[0][0], x[0][x.shape[0] - 1], y[0][0], y[y.shape[0] - 1][0])
    ls = LightSource(azdeg=315, altdeg=35)
    ax.imshow(ls.hillshade(z, vert_exag=1, dx=dx, dy=dy), extent=image_scale, cmap='gray')
    return ax.imshow(ls.shade(z, cmap=matplotlib.cm.gist_earth, blend_mode='overlay', vert_exag=1, dx=dx, dy=dy,
                              vmin=cbar_lim[0], vmax=cbar_lim[1]),
                     extent=image_scale, vmin=cbar_lim[0], vmax=cbar_lim[1], cmap=matplotlib.cm.gist_earth)


def plot_wind_flow(ax, x, y, wx, wy, wvel, **kwargs):
    return ax.streamplot(x, y, wx, wy, density=1, linewidth=1, color='dimgrey')


def plot_wind_quiver(ax, x, y, wx, wy, **kwargs):
    return ax.quiver(x, y, wx, wy, pivot=kwargs.get('pivot','middle'), color=kwargs.get('color','dimgrey'), **kwargs)


def plot_ignition_point(ax, point, **kwargs):
    return ax.plot(*point, linestyle=kwargs.get('linestyle','o'), color=kwargs.get('color','r'),
                   linewidth=kwargs.get('linewidth',5), **kwargs)


class GeoDataDisplay:
    """Draw GeoData information on a matplotlib figure.
    
    'draw_' methods should be called in the order from backgrund to foreground.
    """
    def __init__(self, figure, axis, geodata):
        self._figure, self.axis = figure, axis
        self._geodata = geodata

        x = np.arange(geodata.max_x)
        self._x_ticks = (x * geodata.cell_width) + geodata.x_offset
        y = np.arange(geodata.max_y)
        self._y_ticks = (y * geodata.cell_height) + geodata.y_offset

        self._image_scale = (self._x_ticks[0], self._x_ticks[-1], self._y_ticks[0], self._y_ticks[-1])

        self._x_mesh, self._y_mesh = np.meshgrid(x, y)

        self._drawings = []
        self._colorbars = []

    @classmethod
    def pyplot_figure(cls, geodata):
        figure, axis = get_pyplot_figure_and_axis()
        return cls(figure, axis, geodata)

    def draw_elevation_shade(self, with_colorbar=True, **kwargs):
        shade = plot_elevation_shade(self.axis, self._x_mesh, self._y_mesh,
                                     self._geodata['elevation'].T[::-1, ...],
                                     dx=self._geodata.cell_width, dy=self._geodata.cell_height,
                                     image_scale=self._image_scale, **kwargs)
        self._drawings.append(shade)
        if with_colorbar:
            self._add_elevation_shade_colorbar(shade)

    def _add_elevation_shade_colorbar(self, shade):
        cb = self._figure.colorbar(shade, ax=self.axis, shrink=0.65, aspect=20)
        cb.set_label("Height [m]")
        self._colorbars.append(cb)

    def draw_wind_quiver(self, **kwargs):
        wind_vel = self._geodata['wind_velocity']
        wind_ang = self._geodata['wind_angle']
        WX = wind_vel * np.cos(wind_ang)
        WY = wind_vel * np.sin(wind_ang)

        self._drawings.append(plot_wind_quiver(self.axis, *np.meshgrid(self._x_ticks[::10], self._y_ticks[::10]),
                                               WX[::10, ::10], WY[::10, ::10], **kwargs))

    def draw_ignition_contour(self, with_labels=True, **kwargs):
        # mask all cells whose ignition has not been computed
        igni = np.array(self._geodata['ignition'])
        igni[igni >= np.finfo(np.float64).max] = np.nan

        # plot fire front with contour lines in minutes
        self._drawings.append(plot_ignition_contour(self.axis, self._x_ticks, self._y_ticks, igni.T / 60.),
                              **kwargs)

        if with_labels:
            self._add_ignition_contour_labels()

    def _add_ignition_contour_labels(self, **kwargs):
        self.axis.clabel(self._drawings[-1], inline=True, fontsize='smaller', inline_spacing=1, linewidth=2, fmt='%.0f')

    def draw_ignition_shade(self, with_colorbar=True, **kwargs):
        # mask all cells whose ignition has not been computed
        igni = np.array(self._geodata['ignition'])
        igni[igni >= np.finfo(np.float64).max] = np.nan

        shade = plot_ignition_shade(self.axis, self._x_mesh, self._y_mesh, np.around(igni.T[::-1, ...]/60., 1),
                                    dx=self._geodata.cell_width, dy=self._geodata.cell_height,
                                    image_scale=self._image_scale, **kwargs)
        self._drawings.append(shade)
        if with_colorbar:
            self._add_ignition_shade_colorbar(shade)

    def _add_ignition_shade_colorbar(self, shade, **kwargs):
        cb = self._figure.colorbar(shade, ax=self.axis, shrink=0.65, aspect=20)
        cb.set_label("Ignition time [min]")
        self._colorbars.append(cb)


# "DisplayExtension" classes attach new member functions to a GeoDataDisplay to make it able to display other data
class DisplayExtension(metaclass=abc.ABCMeta):

    @abc.abstractmethod
    def extend(self, geodatadisplay):
        pass


class IgnitionPointDisplayExtension(DisplayExtension):
    """Extension to GeoDataDisplay that draws ignition points"""

    def __init__(self, ignition_points):
        self.ignition_points = ignition_points

    def extend(self, geodatadisplay):
        '''Bounds draw_ignition_points to a GeoDataDisplayInstance.'''
        geodatadisplay.ignition_points = self.ignition_points
        geodatadisplay.draw_ignition_points = types.MethodType(IgnitionPointDisplayExtension._draw_ignition_points_extension, geodatadisplay)

    def _draw_ignition_points_extension(self, *args, **kwargs):
        '''Draw ignition point in a GeoDataDisplay figure.'''
        for p in self.ignition_points:
            self.drawings = plot_ignition_point(self.axis, p)


class UAVDisplayExtension(DisplayExtension):
    """Extension to GeoDataDisplay that draws small uavs"""

    def __init__(self, uav_state):
        self.uav_state = uav_state

    def extend(self, geodatadisplay):
        '''Bounds draw_ignition_points to a GeoDataDisplayInstance.'''
        geodatadisplay.uav_state = self.uav_state
        geodatadisplay.draw_uav = types.MethodType(UAVDisplayExtension._draw_uav_extension, geodatadisplay)

    def _draw_uav_extension(self, *args, **kwargs):
        '''Draw ignition point in a GeoDataDisplay figure.'''
        for p in self.uav_state:
            self.drawings = plot_uav(self.axis, self.uav_state, args, kwargs)


def plot3d_elevation_shade(ax, x, y, z, dx=25, dy=25, **kwargs):
    ls = LightSource(azdeg=120, altdeg=45)
    rgb = ls.shade(z, cmap=matplotlib.cm.terrain, vert_exag=0.1, blend_mode='overlay')
    return ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5, linewidth=0, antialiased=True, shade=True)


def plot3d_wind_arrows(ax, x, y, z, wx, wy, wz, **kwargs):
    return ax.quiver(x, y, z, wx, wy, wz, pivot='middle', cmap=matplotlib.cm.viridis)