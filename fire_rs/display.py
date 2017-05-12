from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from matplotlib import cm


def get_default_figure_and_axis():
    fire_fig = plt.figure()
    fire_ax = fire_fig.gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

    ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
    fire_ax.yaxis.set_major_formatter(ax_formatter)
    fire_ax.xaxis.set_major_formatter(ax_formatter)
    return fire_fig, fire_ax


def plot_firefront_contour(ax, x, y, firefront, nfronts=20):
    fronts = ax.contour(x, y, firefront, nfronts, cmap=cm.Set1)
    labels = ax.clabel(fronts, inline=True, fontsize='smaller', inline_spacing=1, linewidth=2, fmt='%.0f')
    return fronts, labels



def plot_elevation_contour(ax, x, y, z):
    contour = ax.contour(x, y, z, 15, cmap=cm.gist_earth)
    labels = plt.clabel(contour, inline=1, fontsize=10)
    return contour, labels


def plot_elevation_shade(ax, x, y, z, dx=25, dy=25):
    cbar_lim = (z.min(), z.max())

    image_scale = (x[0][0], x[0][x.shape[0] - 1], y[0][0], y[y.shape[0] - 1][0])
    ls = LightSource(azdeg=315, altdeg=45)
    ax.imshow(ls.hillshade(z, vert_exag=5, dx=dx, dy=dy), extent=image_scale, cmap='gray')
    return ax.imshow(ls.shade(z, cmap=cm.terrain, blend_mode='overlay', vert_exag=1, dx=dx, dy=dy,
                              vmin=cbar_lim[0], vmax=cbar_lim[1]),
                     extent=image_scale, vmin=cbar_lim[0], vmax=cbar_lim[1], cmap=cm.terrain)


def plot_wind_flow(ax, x, y, wx, wy, wvel):
    return ax.streamplot(x, y, wx, wy, density=1, linewidth=1, color='dimgrey')


def plot_wind_arrows(ax, x, y, wx, wy):
    return ax.quiver(x, y, wx, wy, pivot='middle', color='dimgrey')


def plot3d_elevation_shade(ax, x, y, z, dx=25, dy=25):
    ls = LightSource(azdeg=120, altdeg=45)
    rgb = ls.shade(z, cmap=cm.terrain, vert_exag=0.1, blend_mode='overlay')
    return ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5, linewidth=0, antialiased=True, shade=True)

def plot3d_wind_arrows(ax, x, y, z, wx, wy, wz):
    return ax.quiver(x, y, z, wx, wy, wz, pivot='middle', cmap=cm.viridis)
