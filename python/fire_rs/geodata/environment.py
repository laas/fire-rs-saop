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

import logging
import os
import numbers
import numpy as np

import fire_rs.firemodel.environment as fire_env

from fire_rs.geodata.elevation import ElevationMap, ElevationTile
from fire_rs.geodata.landcover import LandCoverMap, LandCoverTile
from fire_rs.geodata.wind import WindMap, WindNinjaCLI
from fire_rs.geodata.geo_data import GeoData, Area

logger = logging.getLogger(__name__)

# Set default data folders as the one given in the environment variable $FIRERS_DATA or
# to Rafael's home if not set
# TODO: check whether it is useful/desirable to keep the internal folder
DEFAULT_FIRERS_DATA_FOLDER = os.environ['FIRERS_DATA'] \
    if 'FIRERS_DATA' in os.environ else '/home/rbailonr/firers_data'
DEFAULT_FIRERS_DEM_DATA = os.path.join(DEFAULT_FIRERS_DATA_FOLDER,
                                       'dem/BDALTIV2_2-0_25M_TIF_LAMB93-IGN69_D031_2016-11-17')
DEFAULT_FIRERS_WIND_DATA = os.path.join(DEFAULT_FIRERS_DATA_FOLDER,
                                        'wind/BDALTIV2_2-0_25M_TIF_LAMB93-IGN69_D031_2016-11-17')
DEFAULT_FIRERS_LANDCOVER_DATA = os.path.join(DEFAULT_FIRERS_DATA_FOLDER,
                                             'landcover/BDALTIV2_2-0_25M_TIF_LAMB93-IGN69_D031_2016-11-17')

CORINELANDCOVER_TO_FUELMODEL_REMAP = {1: 'NB1', 2: 'NB1', 3: 'NB1', 4: 'NB1', 5: 'NB1',
                                      6: 'NB1', 7: 'NB1', 8: 'NB1', 9: 'NB1', 10: 'NB1', 11: 'NB1',
                                      12: 'GR4',
                                      # Non-irrigated arable land -> Moderate Load, Dry Climate Grass
                                      13: 'NB8',
                                      14: 'NB8',
                                      15: 'SH2',  # Vineyards -> Moderate Load Dry Climate Shrub
                                      16: 'SH2',  # Olive groves -> Moderate Load Dry Climate Shrub
                                      17: 'SH2',
                                      # Fruit trees and berry plantations -> Moderate Load Dry Climate Shrub
                                      18: 'GR8',
                                      # Pastures -> High Load, Very Coarse, Humid Climate Grass
                                      19: 'SH5',  # No actual reason for this relation
                                      20: 'SH7',  # No actual reason for this relation
                                      21: 'SH5',  # No actual reason for this relation
                                      22: 'SH7',  # No actual reason for this relation
                                      23: 'TL6',
                                      # Broad-leaved forest -> Moderate Load Broadleaf Litter
                                      24: 'TL8',  # Coniferous forest -> Long-Needle Litter
                                      25: 'TU1',
                                      # Mixed forest -> Low Load Dry Climate Timber-Grass-Shrub (Doesn't correspond in fact)
                                      26: 'GR9',
                                      # Natural grasslands -> Very High Load, Humid Climate Grass
                                      27: 'GS4',
                                      # Moors and heathland -> High Load, Humid Climate Grass-Shrub
                                      28: 'SH7',
                                      29: 'SH4',
                                      30: 'NB9',
                                      31: 'NB9',
                                      32: 'GR1',
                                      # Sparsely vegetated areas -> Short, Sparse Dry Climate Grass
                                      33: 'NB9',
                                      34: 'NB9',
                                      35: 'NB2',
                                      36: 'NB8', 37: 'NB8', 38: 'NB8', 39: 'NB8', 40: 'NB8',
                                      41: 'NB8', 42: 'NB8',
                                      43: 'NB8', 44: 'NB8', 48: 'NB9', 49: 'NB9', 50: 'NB8',
                                      255: 'NB9'}

_C_FUEL = 'SH7'  # Very High Load, Dry Climate Shrub
CONSTANT_FUELMODEL_REMAP = {1: 'NB1', 2: 'NB1', 3: 'NB1', 4: 'NB1', 5: 'NB1',
                            6: 'NB1', 7: 'NB1', 8: 'NB1', 9: 'NB1', 10: 'NB1',
                            11: 'NB1', 12: _C_FUEL, 13: _C_FUEL, 14: _C_FUEL, 15: _C_FUEL,
                            16: _C_FUEL, 17: _C_FUEL, 18: _C_FUEL, 19: _C_FUEL, 20: _C_FUEL,
                            21: _C_FUEL, 22: _C_FUEL, 23: _C_FUEL, 24: _C_FUEL, 25: _C_FUEL,
                            26: _C_FUEL, 27: _C_FUEL, 28: _C_FUEL, 29: _C_FUEL, 30: 'NB9',
                            31: 'NB9', 32: _C_FUEL, 33: 'NB9', 34: 'NB2', 35: 'NB8',
                            36: 'NB8', 37: 'NB8', 38: 'NB8', 39: 'NB8', 40: 'NB8',
                            41: 'NB8', 42: 'NB8', 43: 'NB8', 44: 'NB8', 48: 'NB9',
                            49: 'NB9', 50: 'NB8', 255: 'NB9'}

EVERYTHING_FUELMODEL_REMAP = {1: _C_FUEL, 2: _C_FUEL, 3: _C_FUEL, 4: _C_FUEL, 5: _C_FUEL,
                              6: _C_FUEL, 7: _C_FUEL, 8: _C_FUEL, 9: _C_FUEL, 10: _C_FUEL,
                              11: _C_FUEL, 12: _C_FUEL, 13: _C_FUEL, 14: _C_FUEL, 15: _C_FUEL,
                              16: _C_FUEL, 17: _C_FUEL, 18: _C_FUEL, 19: _C_FUEL, 20: _C_FUEL,
                              21: _C_FUEL, 22: _C_FUEL, 23: _C_FUEL, 24: _C_FUEL, 25: _C_FUEL,
                              26: _C_FUEL, 27: _C_FUEL, 28: _C_FUEL, 29: _C_FUEL, 30: _C_FUEL,
                              31: _C_FUEL, 32: _C_FUEL, 33: _C_FUEL, 34: _C_FUEL, 35: _C_FUEL,
                              36: _C_FUEL, 37: _C_FUEL, 38: _C_FUEL, 39: _C_FUEL, 40: _C_FUEL,
                              41: _C_FUEL, 42: _C_FUEL, 43: _C_FUEL, 44: _C_FUEL, 48: _C_FUEL,
                              49: _C_FUEL, 50: _C_FUEL, 255: _C_FUEL}


class World:
    """Class providing access to environment data."""

    def __init__(self, elevation_path=DEFAULT_FIRERS_DEM_DATA, wind_path=DEFAULT_FIRERS_WIND_DATA,
                 landcover_path=DEFAULT_FIRERS_LANDCOVER_DATA,
                 landcover_to_fuel_remap=CONSTANT_FUELMODEL_REMAP, wind_mesh_resolution='fine'):
        self._elevation_path = os.path.abspath(elevation_path)
        self._elevation_map = ElevationMap([])
        self._load_elevation_tiles()

        self._landcover_path = os.path.abspath(landcover_path)
        self._landcover_map = LandCoverMap([])
        self._load_landcover_tiles()

        self._default_landcover_to_fuel_remap = landcover_to_fuel_remap

        self._wind_path = os.path.abspath(wind_path)
        self._wind_maps = {'domainAverageInitialization': {},
                           'pointInitialization': {},
                           'wxModelInitialization': {}}
        self.dem_wind_tile_split = 1

        # Wind maps is a collection of wind scenarios, sorted by initialization method.
        # For domainAverageInitialization maps are sorted by (speed, direction)

        # Output resolution must be the same as the DEM resolution for this world.
        # However, the mesh resolution for windninja has to be quite coarse in order to have
        # simulations completed in reasonable time.
        pixel_res = self._elevation_map.pixel_size
        mesh_resolution = pixel_res if not wind_mesh_resolution else wind_mesh_resolution
        cli_arguments = {}
        if isinstance(mesh_resolution, str):
            cli_arguments['mesh_choice'] = mesh_resolution
        elif isinstance(mesh_resolution, numbers.Number):
            cli_arguments['mesh_resolution'] = mesh_resolution

        domain_scenario = WindNinjaCLI(ascii_out_resolution=pixel_res,
                                       cli_arguments=cli_arguments)
        domain_scenario.add_arguments(**WindNinjaCLI.domain_average_args(0, 0))
        domain_scenario.add_arguments(**WindNinjaCLI.output_type_args())
        domain_scenario.set_output_path(self._wind_path)
        self._windninja_domain = domain_scenario

    def _load_elevation_tiles(self):
        for f in os.scandir(self._elevation_path):
            if f.is_file():
                if f.name.endswith(".aux.xml"):
                    continue
                try:
                    tile = ElevationTile(f.path)
                    self._elevation_map.add_tile(tile)
                except RuntimeError as e:
                    logger.exception("RuntimeError while loading elevation tile from %s", f.path)
                except AttributeError as e:
                    logger.exception("AttributeError while loading elevation tile from %s", f.path)

    def _load_landcover_tiles(self):
        for f in os.scandir(self._landcover_path):
            if f.is_file():
                if f.name.endswith(".aux.xml"):
                    continue
                try:
                    tile = LandCoverTile(f.path)
                    self._landcover_map.add_tile(tile)
                except RuntimeError as e:
                    logger.exception("RuntimeError while loading landcover tile from %s", f.path)
                except AttributeError as e:
                    logger.exception("AttributeError while loading landcover tile from %s", f.path)

    def get_fuel_type(self, position, remap=None) -> 'GeoData':
        """Retrieves the fuel type of a given point/area of the map.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]
        """

        if remap is None:
            remap = self._default_landcover_to_fuel_remap

        landcover = self.get_landcover_class(position)
        if isinstance(landcover, numbers.Number):
            return remap[landcover]

        landcover_array = landcover['landcover']
        landcover_array_copy = landcover_array.copy()
        for k, v in remap.items():
            landcover_array_copy[landcover_array == k] = fire_env.get_fuel_model_id(v)
        return landcover.clone(data_array=landcover_array_copy, dtype=[('fuel', 'int32')])

    def get_landcover_class(self, position) -> 'GeoData':
        """Retrieves the land cover class of a given point/area of the map.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]
        """
        assert len(position) == 2, "Need coordinates for x and y"

        if isinstance(position[0], numbers.Number) and isinstance(position[1],
                                                                  numbers.Number):  # point
            return self._landcover_map.get_class(position)
        else:  # position is a rectangle
            assert len(position[0]) == 2 and len(position[1]) == 2
            return self._landcover_map.get_values(position)

    def get_elevation(self, position) -> 'GeoData':
        """Retrieves elevation of a given point/area of the map.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]
        """
        assert len(position) == 2, "Need coordinates for x and y"

        if isinstance(position[0], numbers.Number) and isinstance(position[1],
                                                                  numbers.Number):  # point
            return self._elevation_map.get_elevation(position)
        else:  # position is a rectangle
            assert len(position[0]) == 2 and len(position[1]) == 2
            return self._elevation_map.get_values(position)

    def get_slope(self, area) -> 'GeoData':
        """Returns a GeoData containing the slope percentage and raise direction of an area."""
        ((x_min, x_max), (y_min, y_max)) = area

        # extract DEM on a slightly large area to avoid border effects
        dem = self.get_elevation([[x_min - self._elevation_map.pixel_size,
                                   x_max + self._elevation_map.pixel_size],
                                  [y_min - self._elevation_map.pixel_size,
                                   y_max + self._elevation_map.pixel_size]])
        z = dem.data.view(np.float64)
        assert dem.data.shape == z.shape, 'Apparently, the returned DEM is not an array of float'

        def rolled(x_roll, y_roll):
            """Returns a view of the DEM array rolled on X/Y axis"""
            return np.roll(np.roll(z, x_roll, axis=0), y_roll, axis=1)

        # compute elevation change on x and y direction, cf:
        # http://desktop.arcgis.com/fr/arcmap/10.3/tools/spatial-analyst-toolbox/how-slope-works.htm
        dzdx = rolled(-1, -1) + 2 * rolled(-1, 0) + rolled(-1, 1) - \
               rolled(1, -1) - 2 * rolled(1, 0) - rolled(1, -1)
        dzdx /= (8 * dem.cell_width)
        dzdy = rolled(1, 1) + 2 * rolled(0, 1) + rolled(-1, 1) - \
               rolled(1, -1) - 2 * rolled(0, -1) - rolled(-1, -1)
        dzdy /= (8 * dem.cell_width)

        # get percentage of slope and the direction of raise and save them as GeoData
        slope_percent = np.sqrt(np.power(dzdx, 2) + np.power(dzdy, 2)) * 100
        raise_dir = np.arctan2(dzdy, dzdx)
        sp = dem.clone(np.array(slope_percent, dtype=[('slope', 'float64')]))
        rd = dem.clone(np.array(raise_dir, dtype=[('raise_dir', 'float64')]))

        # combine slope and raise direction into one GeoData and fit it to the area originally asked
        return sp.combine(rd).subset(Area(x_min, x_max, y_min, y_max))

    def get_wind(self, position, **kwargs) -> 'GeoData':
        """Get wind for a specific wind scenario.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]

        keyword arguments:
            domain_average: (speed, direction)
                speed in km/h (it is rounded to units precision)
                direction in radians (Then is rounded to degree precision)
        """
        assert len(position) == 2, "Need coordinates for x and y"
        dom_av = kwargs.get('domain_average')
        if dom_av is None:
            raise TypeError("Only domainAverageInitialization method is implemented.")
        else:
            from .wind import trigo_angle_to_geo_angle
            assert -np.pi <= dom_av[1] <= 2 * np.pi, \
                "Wind direction should be given in radians, where 0 is East to West and rotation is trigonometric"
            # Detect if a map for this wind case has been loaded, and create if not
            dom_av = (
                "{:.0f}".format(dom_av[0]), "{:.0f}".format(trigo_angle_to_geo_angle(dom_av[1])))
            wind_map = self._wind_maps['domainAverageInitialization'].get(dom_av)

            if wind_map is None:  # create a new wind map for this domain average
                windninja = WindNinjaCLI(cli_arguments=self._windninja_domain.args)
                windninja.add_arguments(**{'input_speed': dom_av[0], 'input_direction': dom_av[1]})
                wind_map = WindMap([], self._elevation_map, windninja,
                                   dem_tile_split=self.dem_wind_tile_split)
                self._wind_maps['domainAverageInitialization'][dom_av] = wind_map

            if isinstance(position[0], numbers.Number) and isinstance(position[1],
                                                                      numbers.Number):  # point
                return wind_map.get_wind(position)
            else:  # position is a rectangle
                assert len(position[0]) == 2 and len(position[1]) == 2
                return wind_map.get_values(position)
