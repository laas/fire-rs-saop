"""Main client of fire_rs library"""

from fire_rs.geodata import environment
import fire_rs.firemodel.propagation as propagation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib import cm
import numpy as np

area = [[530000.0, 540000.0], [6230000.0, 6240000.0]]
ignition_point = (200, 250)
area_wind = (4, 180)

# Obtain geographic data of defined area
world = environment.World()
some_area = world.get_elevation(area)
some_area_wind = world.get_wind(area, domain_average=area_wind)
some_area_slope = world.get_slope(area)

x = np.arange(some_area.data.shape[0])
y = np.arange(some_area.data.shape[0])
X, Y = np.meshgrid(x, y)
Z = some_area.data['elevation']

# Light a fire and propagate it over the whole area
env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
ignitions = propagation.propagate(env, *ignition_point)
ignitions.write_to_file('/tmp/igni.tif')


def plot_firefront_contour(ax, x, y, firefront, nfronts=20):
    return ax.contour(x, y, firefront, 25, cmap=cm.magma)


def plot_elevation_contour(ax, x, y, z):
    contour = ax.contour(x, y, z, 15, cmap=cm.gist_earth)
    plt.clabel(contour, inline=1, fontsize=10)


def plot_elevation_shade(ax, x, y, z, dx=25, dy=25):
    ls = LightSource(azdeg=315, altdeg=45)
    ax.imshow(ls.hillshade(z, vert_exag=1, dx=dx, dy=dy), cmap='gray')
    ax.imshow(ls.shade(z, cmap=cm.terrain, blend_mode='overlay', vert_exag=1, dx=dx, dy=dy), vmin=0, vmax=z.max())


def plot_wind_flow(ax, x, y, wx, wy, wvel):
    ax.streamplot(x, y, wx, wy, density=1, linewidth=1)  # , cmap=plt.cm.autumn)


def plot_wind_arrows(ax, x, y, wx, wy):
    ax.quiver(x, y, wx, wy, pivot='middle', cmap=cm.viridis)


def plot3d_elevation_shade(ax, x, y, z, dx=25, dy=25):
    ls = LightSource(azdeg=120, altdeg=45)
    rgb = ls.shade(z, cmap=cm.terrain, vert_exag=0.1, blend_mode='overlay')
    ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5,linewidth=0, antialiased=True, shade=True)


def plot3d_wind_arrows(ax, x, y, z, wx, wy, wz):
    ax.quiver(x, y, z, wx, wy, wz, pivot='middle', cmap=cm.viridis)


def plotting_firefront_2d():
    fire_fig = plt.figure()
    fire_ax = fire_fig.gca(aspect='equal')

    plot_elevation_shade(fire_ax, X, Y, Z)
    fronts = plot_firefront_contour(fire_ax, X, Y, ignitions.data['ignition']/60)
    fire_ax.clabel(fronts, inline=True, fontsize='smaller', inline_spacing=1, linewidth=2, fmt='%.0f')
    fire_ax.plot(ignition_point[1], ignition_point[0], 'or', linewidth=5)
    cbar = fire_fig.colorbar(fronts, shrink=0.5, aspect=5)
    cbar.set_label("Ignition time [min]")

    wind_vel = some_area_wind['wind']['wind_velocity']
    wind_ang = some_area_wind['wind']['wind_angle']
    WX = wind_vel * np.cos(wind_ang / 180 * np.pi)
    WY = wind_vel * np.sin(wind_ang / 180 * np.pi)

    plot_wind_arrows(fire_ax, *np.meshgrid(x[::5], y[::5]), WX[::5, ::5], WY[::5, ::5])

    fire_fig.show()


def plotting_wind_2d():
    wind_fig = plt.figure()
    wind_ax = wind_fig.gca(aspect='equal')

    wind_vel = some_area_wind['wind']['wind_velocity']
    wind_ang = some_area_wind['wind']['wind_angle']
    WX = wind_vel * np.cos(wind_ang / 180 * np.pi)
    WY = wind_vel * np.sin(wind_ang / 180 * np.pi)

    plot_elevation_shade(wind_ax, X, Y, Z)
    plot_wind_arrows(wind_ax,*np.meshgrid(x[::5], y[::5]), WX[::5, ::5], WY[::5,::5])

    wind_fig.show()


def plotting_3d():
    elev_fig = plt.figure()
    wind_ax = elev_fig.gca(projection='3d')

    wind_vel = some_area_wind['wind']['wind_velocity']
    wind_ang = some_area_wind['wind']['wind_angle']
    slope_slope = some_area_slope['slope']['slope']
    slope_dir = some_area_slope['slope']['raise_dir']
    WX = wind_vel * np.cos(wind_ang / 180 * np.pi)
    WY = wind_vel * np.sin(wind_ang / 180 * np.pi)
    WZ = slope_slope


    # plot3d_elevation_shade(wind_ax, X, Y, Z)
    plot3d_wind_arrows(wind_ax, *np.meshgrid(x[::5], y[::5], Z[::5]), WX[::5, ::5], WY[::5, ::5], WZ[::5, ::5])

    wind_ax.view_init(elev=10., azim=120)
    elev_fig.show()


plotting_firefront_2d()
plotting_3d()


input()

