from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib
import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms

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


def plot_uav(ax, uav_state, size=1, facecolor='blue', edgecolor='black', **kwargs):
    plane_vertices = (np.array([[3.5, 6], [4, 5], [4, 4], [7, 4], [7, 3],
                      [4,   3], [4, 1], [5, 1], [5, 0], [2, 0],
                      [2, 1], [3, 1], [3, 3], [0, 3], [0, 4],
                      [3, 4], [3, 5]]) - np.array([3.5, 3])) * size
    R = np.array([[np.cos(-uav_state[2]+np.pi/2), -np.sin(-uav_state[2]+np.pi/2)],
                  [np.sin(-uav_state[2]+np.pi/2), np.cos(-uav_state[2]+np.pi/2)]])

    plane_polygon = matplotlib.patches.Polygon(np.matmul(plane_vertices, R) + uav_state[0:2], closed=True, fill=True,
                                               facecolor=facecolor, edgecolor=edgecolor, **kwargs)
    return ax.add_patch(plane_polygon)


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


if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    Lx = 401
    Ly = 401
    Nx = 401
    Ny = 401
    Nt = 200

    import os
    from fire_rs.geodata import environment
    import fire_rs.firemodel.propagation as propagation
    import fire_rs.geodata.geo_data

    def burn(area_bounds, ignition_point, wind):
        """Burn some area from an ignition point with given wind conditions"""
        env = propagation.Environment(area_bounds, wind_speed=wind[0], wind_dir=wind[1])
        f_propagation = propagation.propagate(env, *ignition_point, horizon=5*3600)  # limit propagation to 5 hours
        f_propagation.plot(blocking=False)
        return f_propagation.ignitions()

    area = [[480060.0, 490060.0], [6210074.0, 6220074.0]]
    ignition_point = (10, 20)
    area_wind = (4.11, 3/4*np.pi)
    ignition_times = None
    if not os.path.exists('/tmp/ignition.tif'):
        ignition_times = burn(area, ignition_point, area_wind)
        ignition_times.write_to_file('/tmp/ignition.tif')
    else:
        ignition_times = fire_rs.geodata.geo_data.GeoData.load_from_file('/tmp/ignition.tif')

    world = environment.World()
    some_area = world.get_elevation(area)

    # Generate grid for plotting
    x = np.linspace(0, Lx, Nx)
    y = np.linspace(0, Ly, Ny)
    x, y = np.meshgrid(x, y)

    fig = plt.figure()
    ax = plt.axes(xlim=(0, Lx), ylim=(0, Ly))
    plt.xlabel(r'x')
    plt.ylabel(r'y')

    z = ignition_times.data_display['ignition'] / 60
    print(z)

    # animation function
    def animate(i):
        print("frame {}".format(i))
        cont = plt.contourf(x, y, z, [0, i+1, i+10])
        plt.title(r't = %i' % i)

        return cont

    anim = animation.FuncAnimation(fig, animate, frames=Nt)
    anim.save('animation.mp4')
    plt.show()