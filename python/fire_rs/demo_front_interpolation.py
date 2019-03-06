"""Test fire front interpolation"""

#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#
#
import itertools
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm
import skimage.io
import skimage.draw
import skimage.measure
import scipy.interpolate

import fire_rs.geodata.environment as g_environment
import fire_rs.geodata.display as display

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint

################################################################################
# World initialization

FIRERS_DATA_FOLDER = '/home/rbailonr/firers_data_porto'
FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER, 'dem')
FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER, 'wind')
FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER, 'landcover')

the_world = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                wind_path=FIRERS_WIND_DATA,
                                landcover_path=FIRERS_LANDCOVER_DATA, wind_mesh_resolution='fine',
                                landcover_to_fuel_remap=g_environment.SLOW_FUELMODEL_REMAP)
the_world.dem_wind_tile_split = 1

area = ((2776825.0 - 2500, 2776825.0 + 2500), (2212175.0 - 2500, 2212175.0 + 4500))
ignition = TimedPoint(2776825.0, 2212175.0, 0)

################################################################################
# Reference fire propagation
fire_env = Environment(area, 5., np.pi/2, the_world)

fire_prop = FirePropagation(fire_env)
fire_prop.set_ignition_point(ignition)

propagation_end_time = 60 * 60 * 60
propagation_end_time = np.inf

fire_prop.propagate(propagation_end_time)

################################################################################
# Reference fire display
# Figure terrain + ignition contour + ignition point
gdd = display.GeoDataDisplay.pyplot_figure(
    fire_env.raster.combine(fire_prop.ignitions().slice(["ignition"])), frame=(0., 0.))
gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
gdd.draw_wind_quiver()
gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_points(ignition)

gdd.figure.show()

################################################################################
# Contour extraction
firemap = fire_prop.ignitions()

fire_image = np.ones(firemap.data.shape, dtype=np.float64) * np.NaN
firepoints = {}
# The ignition point is knwon
firepoints[fire_prop.ignitions().array_index((ignition[0], ignition[1]))] = ignition[2]

# Create contour with scikit-image
contours1 = skimage.measure.find_contours(firemap.data["ignition"], 10 * 60 * 60)
contours2 = skimage.measure.find_contours(firemap.data["ignition"], 20 * 60 * 60)
contours3 = skimage.measure.find_contours(firemap.data["ignition"], 30 * 60 * 60)
contours4 = skimage.measure.find_contours(firemap.data["ignition"], 40 * 60 * 60)
contours5 = skimage.measure.find_contours(firemap.data["ignition"], 50 * 60 * 60)
contours6 = skimage.measure.find_contours(firemap.data["ignition"], 60 * 60 * 60)

# Print contour as binary image
comp_cycle = itertools.cycle([lambda x, y: x < y, lambda x, y: x > y])
comp = next(comp_cycle)
for i in [contours1,  contours3,   contours5]:
    comp = next(comp_cycle)
    for contour in i:
        for pt_i in range(len(contour)):
            if comp(pt_i, len(contour) / 2):
                continue

            if pt_i % 1 == 0:
                rr, cc = skimage.draw.line(*np.asarray(contour[pt_i - 1], dtype=int),
                                           *np.asarray(contour[pt_i], dtype=int))
                fire_image[rr, cc] = firemap.data["ignition"][rr[0], cc[0]]
                for r, c in zip(rr, cc):
                    firepoints[r, c] = firemap.data["ignition"][r, c]

fig = plt.figure()
ax = fig.gca()
imag = ax.imshow(fire_image/60.)
fig.colorbar(imag, ax=ax, shrink=0.65, aspect=20, format="%d minutes")
fig.show()

################################################################################
# Interpolate contour
x, y = list(zip(*firepoints.keys()))
z = tuple(firepoints.values())
function = 'thin_plate'
# function = 'linear'
function = 'multiquadric'
# function = 'cubic'
# --function = lambda a: np.sin(a)

# Wildland fire modeling with an Eulerian level set method and automated calibration
# might give a clue of which kind of kernel function to use

zfun_smooth_rbf = scipy.interpolate.Rbf(x, y, z, function=function, epsilon=None,
                                        smooth=0)  # default smooth=0 for interpolation

xi = np.linspace(0, firemap.data.shape[0] - 1, firemap.data.shape[0])
yi = np.linspace(0, firemap.data.shape[1] - 1, firemap.data.shape[1])
meshgrid = np.meshgrid(xi, yi, indexing="ij")

z_dense_smooth_rbf = zfun_smooth_rbf(
    *[x.flatten() for x in meshgrid])  # not really a function, but a callable class instance

z_dense_smooth_rbf = z_dense_smooth_rbf.reshape(len(xi), len(yi))

################################################################################
# Display interpolation
fig = plt.figure()
ax = fig.gca()
levels = list(range(0, 70 * 60 * 60, 10 * 60 * 60))
ax.imshow(fire_image)
c1 = ax.contour(z_dense_smooth_rbf, levels=levels)
ax.clabel(c1)
c2 = ax.contour(firemap.data["ignition"], levels=levels, alpha=0.6)
# ax.imshow(z_dense_smooth_rbf - firemap.data["ignition"])
ax.clabel(c2)
ax.imshow(fire_image)
fig.show()

################################################################################
# Display error
fig = plt.figure()
ax = fig.gca()
levels = list(range(0, 70 * 60, 10 * 60))

diferencia = z_dense_smooth_rbf - firemap.data["ignition"]
# ax.imshow(firemap.data["ignition"])
# 150 min is 200m for wind 5km/h in wind direction
diff_image = ax.imshow(diferencia/60., cmap=matplotlib.cm.seismic, vmin=-150, vmax=150)
cb = fig.colorbar(diff_image, ax=ax, shrink=0.65, aspect=20, format="%d minutes")
cb.set_label("Interpolation error")
ax.imshow(fire_image, cmap=matplotlib.cm.cool)
c2 = ax.contour(firemap.data["ignition"]/60., levels=levels, cmap=matplotlib.cm.cool)
ax.clabel(c2)

# ax.imshow(fire_image)
fig.show()

################################################################################
# In 3D
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.plot_surface(*meshgrid, z_dense_smooth_rbf, color="blue")
fig.show()

ax = fig.add_subplot(122, projection='3d')
ax.plot_surface(*meshgrid, firemap.data["ignition"], color="red")
fig.show()



print("THE END")
