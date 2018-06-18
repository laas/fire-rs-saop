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

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib
import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms

from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from matplotlib import cm

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    Lx = 201
    Ly = 201
    Nx = 201
    Ny = 201
    Nt = 100

    import os
    from fire_rs.geodata import environment
    import fire_rs.firemodel.propagation as propagation
    import fire_rs.geodata.geo_data


    def burn(area_bounds, ignition_point, wind):
        """Burn some area from an ignition point with given wind conditions"""
        env = propagation.Environment(area_bounds, wind_speed=wind[0], wind_dir=wind[1])
        f_propagation = propagation.propagate_from_points(
            env, (ignition_point[0], ignition_point[1], 0),
            until=120 * 60)
        return f_propagation.ignitions()


    area = ((478500.0, 483500.0), (6210000.0, 6215000.0))
    ignition_point = (area[0][0] + 2500.0, area[1][0] + 2100.0)
    area_wind = (10., np.pi / 4)

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
        i *= 10
        print("frame {}".format(i))
        cont = plt.contourf(x, y, z, [0, i + 1, i + 10])
        plt.title(r't = %i' % i)

        return cont


    anim = animation.FuncAnimation(fig, animate, frames=Nt)
    anim.save('animation.mp4')
    plt.show()
