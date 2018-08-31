"""High precision wildfire simulation"""

import os

import numpy as np
import matplotlib.cm

import fire_rs.geodata.environment as g_environment
import fire_rs.geodata.display as display

from fire_rs.geodata.geo_data import GeoData
from fire_rs.firemodel.propagation import Environment, FirePropagation, TimedPoint

FIRERS_DATA_FOLDER = '/home/rbailonr/firers_data_5m'
FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER,
                               'dem/RGEALTI_MNT_5M_ASC_LAMB93_IGN69')
FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER,
                                'wind/RGEALTI_MNT_5M_ASC_LAMB93_IGN69')
FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER,
                                     'landcover/RGEALTI_MNT_5M_ASC_LAMB93_IGN69')

the_world = g_environment.World(elevation_path=FIRERS_DEM_DATA,
                                wind_path=FIRERS_WIND_DATA,
                                landcover_path=FIRERS_LANDCOVER_DATA, wind_mesh_resolution='fine')
the_world.dem_wind_tile_split = 1

fire_env = Environment(((479998.0 - 2000, 484997.0 + 2000), (6210003.0 - 2000, 6215002.0 + 2000)),
                       5, 0, the_world)

# fire_env = Environment(((470010.0, 485010.0), (6210010.0, 6225010.0)), 5, 0, the_world)
# fire_env = Environment(((574995.0, 575005.0), (6199995.0, 6200005.0)), 5, 0, the_world)
fire_prop = FirePropagation(fire_env)
fire_prop.set_ignition_point(TimedPoint(482010, 6212010.0, 0))
# fire_prop.set_ignition_point(TimedPoint(575000.0, 6200000.0, 0))

propagation_end_time = 10000
export_front = (7000, 7100)

fire_prop.propagate(propagation_end_time)

fire_prop.ignitions().write_to_file("demo_propagation_highres_firemap.tif")

filt_fire = fire_prop.ignitions().clone()
filt_fire.data["ignition"][filt_fire.data["ignition"] < export_front[0]] = 0
filt_fire.data["ignition"][filt_fire.data["ignition"] > export_front[1]] = 0
filt_fire.data["ignition"][filt_fire.data["ignition"].nonzero()] = 65535
filt_fire.write_to_image_file("demo_propagation_highres_firemap_" +
                              str(export_front[0]) + "-" +
                              str(export_front[1]) + ".png",
                              layer_name="ignition")

fire_env.raster.write_to_file("demo_propagation_highres_dem.tif", "elevation")

# Figure terrain + ignition contour + ignition point
gdd = display.GeoDataDisplay.pyplot_figure(
    fire_env.raster.combine(fire_prop.ignitions().slice(["ignition"])),
    frame=(0., 0.))
gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
gdd.draw_wind_quiver()
gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
gdd.draw_ignition_points(TimedPoint(482010, 6212010.0, 0))

gdd.figure.show()

gdd.figure.savefig(".".join(("demo_propagation_highres", "svg")), dpi=150, bbox_inches='tight')
