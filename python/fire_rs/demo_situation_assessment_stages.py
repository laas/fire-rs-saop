"""Produce pictures of the Situation Assessment stages Observation, Interpolation, Prediction"""

#  Copyright (c) 2020, CNRS-LAAS
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
import datetime
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
import fire_rs.rbf
import fire_rs.geodata.wildfire

import fire_rs.monitoring.supersaop

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint
################################################################################
# World initialization

FIRERS_DATA_FOLDER = os.environ["FIRERS_DATA"]
FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER, 'dem')
FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER, 'wind')
FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER, 'landcover')

the_world = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                wind_path=FIRERS_WIND_DATA,
                                landcover_path=FIRERS_LANDCOVER_DATA, wind_mesh_resolution='fine',
                                landcover_to_fuel_remap=g_environment.SLOW_FUELMODEL_REMAP)
the_world.dem_wind_tile_split = 1

area = ((2776825.0 - 200, 2776825.0 + 200), (2212175.0 - 200, 2212175.0 + 200))
ignition = TimedPoint(2776825.0+25, 2212175.0-75, 0)

################################################################################
# Reference fire propagation
fire_env = Environment(area, 2., 3 * np.pi / 4, the_world)

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
# gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
# gdd.draw_wind_quiver()
gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_shade()
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
wf_perimeter = fire_rs.geodata.wildfire.Perimeter(firemap, 7 * 60 * 60)
gdd = display.GeoDataDisplay.pyplot_figure(wf_perimeter.geodata, frame=(0., 0.))
# gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
# gdd.draw_wind_quiver()
# gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_shade()
# gdd.draw_ignition_points(ignition)
plt.axis('off')
gdd.figure.show()
gdd.figure.savefig("situation_assessment_contour.png")


################################################################################
# Use the extracted contour to assess current state
c_ass_wf = fire_rs.monitoring.supersaop.SituationAssessment.WildfireCurrentAssessment(fire_env, {**wf_perimeter.cells, **{fire_env.raster.array_index((ignition[0], ignition[1])): ignition[2]}}, perimeter_time=datetime.datetime.fromtimestamp(5 * 60 * 60))
gdd = display.GeoDataDisplay.pyplot_figure(c_ass_wf.geodata, frame=(0., 0.))
# gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
# gdd.draw_wind_quiver()
# gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_shade()
# gdd.draw_ignition_points(ignition)
plt.axis('off')
gdd.figure.show()
gdd.figure.savefig("situation_assessment_current_state.png")

################################################################################
# Predict future propagation
f_ass_wf = fire_rs.monitoring.supersaop.SituationAssessment.WildfireFuturePropagation(
                fire_env, c_ass_wf.perimeter, {},
                c_ass_wf.geodata, datetime.datetime.fromtimestamp(100*60*60))

gdd = display.GeoDataDisplay.pyplot_figure(f_ass_wf.geodata, frame=(0., 0.))
# gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
# gdd.draw_wind_quiver()
# gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_shade()
# gdd.draw_ignition_points(ignition)
plt.axis('off')
gdd.figure.show()
gdd.figure.savefig("situation_assessment_future_state.png")


print("THE END")
