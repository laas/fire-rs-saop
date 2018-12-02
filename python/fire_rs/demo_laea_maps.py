"""High precision wildfire simulation"""

#  Copyright (c) 2018, CNRS-LAAS
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

import os

import numpy as np
import matplotlib.cm

import fire_rs.geodata.environment as g_environment
import fire_rs.geodata.display as display

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint

FIRERS_DATA_FOLDER = '/home/rbailonr/firers_data_porto'
FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER, 'dem')
FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER, 'wind')
FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER, 'landcover')

the_world = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                wind_path=FIRERS_WIND_DATA,
                                landcover_path=FIRERS_LANDCOVER_DATA, wind_mesh_resolution='fine',
                                landcover_to_fuel_remap=g_environment.CONSTANT_FUELMODEL_REMAP)
the_world.dem_wind_tile_split = 1

area = ((2776825.0 - 1500, 2776825.0 + 1500), (2212175.0 - 500, 2212175.0 + 500))
ignition = TimedPoint(2776825.0, 2212175.0, 0)

fire_env = Environment(area, 0., 0, the_world)

fire_prop = FirePropagation(fire_env)
fire_prop.set_ignition_point(ignition)

propagation_end_time = 20*60*60

fire_prop.propagate(propagation_end_time)

# Figure terrain + ignition contour + ignition point
gdd = display.GeoDataDisplay.pyplot_figure(
    fire_env.raster.combine(fire_prop.ignitions().slice(["ignition"])), frame=(0., 0.))
gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
gdd.draw_wind_quiver()
gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_points(ignition)

gdd.figure.show()
print("THE END")