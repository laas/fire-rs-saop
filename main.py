"""Main client of fire_rs library"""


from fire_rs.geodata import environment
import fire_rs.firemodel.propagation as propagation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from matplotlib import cm, pyplot
import numpy as np


def display():

    area = [[535000.0, 540000.0], [6235000.0, 6240000.0]]
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]

    # area = [[525000.0, 530000.0], [6225000.0, 6230000.0]]  # Produces error!
    # area = [[525000.0, 530000.0], [6225000.0, 6250000.0]]
    ignition_point = (100, 100)
    area_wind = (10, np.pi)

    # Obtain geographic data of defined area
    world = environment.World()
    some_area = world.get_elevation(area)
    some_area_wind = world.get_wind(area, domain_average=area_wind)
    some_area_slope = world.get_slope(area)

    x = np.arange(some_area.data.shape[0])
    x = (x * some_area.cell_width) + some_area.x_offset

    y = np.arange(some_area.data.shape[1])
    y = (y * some_area.cell_height) + some_area.y_offset

    X, Y = np.meshgrid(x, y)
    Z = some_area.data['elevation']

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    ignitions = propagation.propagate(env, *ignition_point).slice(['ignition'])
    ignitions.write_to_file('/tmp/igni.tif')


    def plot_firefront_contour(ax, x, y, firefront, nfronts=20):
        return ax.contour(x, y, firefront, 10, cmap=cm.Set1)


    def plot_elevation_contour(ax, x, y, z):
        contour = ax.contour(x, y, z, 15, cmap=cm.gist_earth)
        plt.clabel(contour, inline=1, fontsize=10)


    def plot_elevation_shade(ax, x, y, z, dx=25, dy=25):
        cbar_lim= (z.min()-0.2*(z.max()-z.min()), z.max()) # This version skips the blue part of the colormap
        cbar_lim= (z.min(), z.max())

        image_scale = (x[0][0], x[0][x.shape[0]-1],  y[0][0],  y[y.shape[0]-1][0])
        ls = LightSource(azdeg=315, altdeg=45)
        ax.imshow(ls.hillshade(z, vert_exag=5, dx=dx, dy=dy), extent=image_scale, cmap='gray')
        return ax.imshow(ls.shade(z, cmap=cm.terrain, blend_mode='overlay', vert_exag=1, dx=dx, dy=dy,
                                  vmin=cbar_lim[0], vmax=cbar_lim[1]),
                         extent=image_scale, vmin=cbar_lim[0], vmax=cbar_lim[1], cmap=cm.terrain)


    def plot_wind_flow(ax, x, y, wx, wy, wvel):
        ax.streamplot(x, y, wx, wy, density=1, linewidth=1, color='dimgrey')


    def plot_wind_arrows(ax, x, y, wx, wy):
        ax.quiver(x, y, wx, wy, pivot='middle', color='dimgrey')


    def plot3d_elevation_shade(ax, x, y, z, dx=25, dy=25):
        ls = LightSource(azdeg=120, altdeg=45)
        rgb = ls.shade(z, cmap=cm.terrain, vert_exag=0.1, blend_mode='overlay')
        ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5,linewidth=0, antialiased=True, shade=True)


    def plot3d_wind_arrows(ax, x, y, z, wx, wy, wz):
        ax.quiver(x, y, z, wx, wy, wz, pivot='middle', cmap=cm.viridis)


    def plotting_firefront_2d():
        fire_fig = plt.figure()
        fire_ax = fire_fig.gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

        ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
        fire_ax.yaxis.set_major_formatter(ax_formatter)
        fire_ax.xaxis.set_major_formatter(ax_formatter)
        shade = plot_elevation_shade(fire_ax, X, Y, Z.T[::-1,...])
        cbar = fire_fig.colorbar(shade, shrink=0.5, aspect=2)

        wind_vel = some_area_wind['wind']['wind_velocity']
        wind_ang = some_area_wind['wind']['wind_angle']
        WX = wind_vel * np.cos(wind_ang)
        WY = wind_vel * np.sin(wind_ang)

        plot_wind_arrows(fire_ax, *np.meshgrid(x[::10], y[::10]), WX[::10, ::10], WY[::10, ::10])
        # plot_wind_flow(fire_ax, *np.meshgrid(x[::10], y[::10]), WY[::10, ::10], WX[::10, ::10], wind_vel[::10, ::10])

        fronts = plot_firefront_contour(fire_ax, X, Y, ignitions.data['ignition'].T[::-1,...]/60)
        fire_ax.clabel(fronts, inline=True, fontsize='smaller', inline_spacing=1, linewidth=2, fmt='%.0f')
        ignition_p = ((ignition_point[0] * some_area.cell_width) + some_area.x_offset,
                      (ignition_point[1] * some_area.cell_height) + some_area.y_offset)
        fire_ax.plot(ignition_p[0], ignition_p[1], 'or', linewidth=5)

        cbar.set_label("Height [m]")

        return fire_fig


    def plotting_3d():
        elev_fig = plt.figure()
        wind_ax = elev_fig.gca(projection='3d')

        wind_vel = some_area_wind['wind']['wind_velocity']
        wind_ang = some_area_wind['wind']['wind_angle']
        slope_slope = some_area_slope['slope']['slope']
        slope_dir = some_area_slope['slope']['raise_dir']
        WX = wind_vel * np.cos(wind_ang)
        WY = wind_vel * np.sin(wind_ang)
        WZ = slope_slope


        plot3d_elevation_shade(wind_ax, X, Y, Z)
        plot_elevation_contour(wind_ax, X, Y, Z)
        # plot_firefront_contour(wind_ax, X, Y, ignitions.data['ignition']/60)
        # plot3d_wind_arrows(wind_ax, *np.meshgrid(x[::5], y[::5], Z[::5]), WX[::5, ::5], WY[::5, ::5], WZ[::5, ::5])

        wind_ax.view_init(elev=10., azim=120)
        elev_fig.show()


    figure = plotting_firefront_2d()
    figure.show()
    # plotting_3d()

def step_by_step():
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    ignition_point = (100, 100)
    ignition_time = 0
    area_wind = (10, np.pi)

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    propag = propagation.FirePropagation(env)
    propag.set_ignition_point(*ignition_point, ignition_time)

    plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

    # todo: axes and directions are wrong when using imshow to display an array (see workarounds in GeoData)
    p = plt.imshow(propag.slice(['ignition']).data.view('float32'))
    plt.title("Wildfire propagation")

    next_step = ignition_time
    while not propag.propagation_finished:
        propag.propagate(next_step)
        next_step += 300
        p.set_data(propag.slice(['ignition']).data.view('float32'))
        plt.pause(0.1)

if __name__ == "__main__":
    # step_by_step()
    display()
    pyplot.show()
