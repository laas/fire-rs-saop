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
import subprocess
import itertools
import shutil

import numpy as np

from fire_rs.geodata.basemap import DigitalMap, RasterTile

logger = logging.getLogger(__name__)

# DEM tiles are to big to allow computing the wind on the whole tile.
# the following factor is used to split the tile on both axis to build
# smaller DEM tiles to feed windninja
_DEFAULT_DEM_TILE_SPLIT = 4

WINDNINJA_CLI_PATH = os.environ['WINDNINJA_CLI_PATH'] \
    if 'WINDNINJA_CLI_PATH' in os.environ else None
if not WINDNINJA_CLI_PATH:
    p = shutil.which("WindNinja_cli")
    if p:
        WINDNINJA_CLI_PATH = os.path.dirname(p)
    else:
        WINDNINJA_CLI_PATH = ""
if not os.path.exists(os.path.join(WINDNINJA_CLI_PATH, 'WindNinja_cli')):
    raise FileNotFoundError("WindNinja_cli can not be found. Please set $WINDNINJA_CLI_PATH.")


def geo_angle_to_trigo_angle(angles):
    """Converts a geographic angle (used by windninja) to a trigonometric angle in radians 
    where 0 is East to West and rotation is trigonometric
    """
    angles = angles - 90.  # rotate to center on X axis
    angles = angles * (2 * np.pi) / 360.  # to radians
    return angles % (np.pi * 2)


def trigo_angle_to_geo_angle(angles):
    """Converts a trigonometric angle to a geographic one (as used by windninja)"""
    angles = angles * 360. / (2 * np.pi)
    angles = angles + 90
    return angles % 360.


class WindMap(DigitalMap):
    """Wind map associated to an elevation map."""

    def __init__(self, tiles, elevation_map, windninja, dem_tile_split=_DEFAULT_DEM_TILE_SPLIT):
        """Initialise WindMap.

        :param tiles: Sequence of WindTiles
        :param elevation_map: Associated ElevationMap.
        :param scenario: WindNinjaCLI instance with all the necessary arguments set.
        """
        super().__init__(tiles)
        self.elevation_map = elevation_map

        self.scenario = windninja.args
        self.windninja_cli = windninja

        self.dem_tile_split = dem_tile_split

        sce_list = []
        if self.scenario['initialization_method'] == 'domainAverageInitialization':
            sce_list.append(str(int(float(self.scenario['input_direction']))))
            sce_list.append(str(int(float(self.scenario['input_speed']))))
            if self.scenario.get('diurnal_winds') is not None and \
                    bool(self.scenario['diurnal_winds']) is True:
                sce_list.append('-'.join([self.scenario['month'],
                                          self.scenario['day'],
                                          self.scenario['year']]))
                sce_list.append(''.join([self.scenario['hour'],
                                         self.scenario['minute']]))

            sce_list.append(''.join([str(self.scenario['ascii_out_resolution']),
                                     self.scenario['units_ascii_out_resolution']]))

        self.scenario_str = '_'.join(sce_list)  # part of WindNinja output file(s)

    def _load_tile(self, position):
        """Load the tile corresponding to this possition.

        It runs windninja on a subset of the DEM if necessary.
        """
        (x, y) = position
        base_tile = self.elevation_map.tile_of_location(position)
        assert (x, y) in base_tile

        # find in which subpart of the elevation tile this location is
        xi = int((x - base_tile.border_x_min) / (
                base_tile.border_x_max - base_tile.border_x_min) * self.dem_tile_split)
        yi = int((y - base_tile.border_y_min) / (
                base_tile.border_y_max - base_tile.border_y_min) * self.dem_tile_split)

        # build tile names of this wind scenario and subpart of the DEM tile
        tile_name = os.path.splitext(os.path.split(base_tile.filenames[0])[1])[0] + \
                    '[{0}%{2},{1}%{2}]'.format(xi, yi, self.dem_tile_split)
        dem_file_name = os.path.join(self.scenario['output_path'], tile_name + '.tif')

        # save smaller DEM tile if it does not exists yet
        if not os.path.exists(dem_file_name):
            dem = base_tile.as_geo_data().split(self.dem_tile_split, 1)[xi].split(
                1, self.dem_tile_split)[yi]
            assert position in dem
            dem.write_to_file(dem_file_name)

        windfile_paths = [os.path.join(self.scenario['output_path'],
                                       '_'.join([tile_name,
                                                 self.scenario_str,
                                                 'vel.asc'])),
                          os.path.join(self.scenario['output_path'],
                                       '_'.join([tile_name,
                                                 self.scenario_str,
                                                 'ang.asc']))]

        if not (os.path.exists(windfile_paths[0]) and os.path.exists(windfile_paths[1])):
            self._run_windninja(dem_file_name)

        # FIXME: Crop asc files to dem col and row count.
        # WindNinja results are bigger than the input

        return WindTile(windfile_paths)

    def _run_windninja(self, elevation_file: str):
        self.windninja_cli.set_elevation_file(elevation_file)
        completed = self.windninja_cli.run()
        if completed.returncode != 0:
            raise RuntimeError("Error during execution! WindNinja returned {}.".format(
                completed.returncode))

    def get_value(self, position):
        """Get the value corresponding to a position."""
        if position not in self.elevation_map:
            raise KeyError("Location out of elevation map bounds")

        if position not in self:
            # We have to load a map for this position
            tile = self._load_tile(position)
            self.add_tile(tile)
            return tile[position]
        return super().get_value(position)

    def get_values(self, positions_intervals):
        ((x_min, x_max), (y_min, y_max)) = positions_intervals
        assert all(
            [p in self.elevation_map for p in itertools.product((x_min, x_max), (y_min, y_max))]), \
            'The requested rectangle is not contained in the known DEM tiles'
        sample_tile = self.elevation_map.tile_of_location((x_min, y_min))
        subtile_width = \
            (sample_tile.x_max - sample_tile.x_min + sample_tile.x_delta) / self.dem_tile_split

        # round x/y to cell centers
        (x_min, y_min) = self.elevation_map.tile_of_location(
            (x_min, y_min)).nearest_projected_point((x_min, y_min))
        (x_max, y_max) = self.elevation_map.tile_of_location(
            (x_max, y_max)).nearest_projected_point((x_max, y_max))

        # sample xs and ys so we have one in each subcell
        xs = list(range(int(x_min), int(x_max), int(subtile_width))) + [int(x_max)]
        ys = list(range(int(y_min), int(y_max), int(subtile_width))) + [int(y_max)]

        # force loading all subtiles
        for pos in itertools.product(xs, ys):
            self.get_value(pos)

        # now that all tiles are loaded, rely on the generic method
        return super().get_values(positions_intervals)

    def get_wind(self, position):
        """Get the wind vector of a position."""
        return self.get_value(position)


class WindTile(RasterTile):

    def __init__(self, windfile_paths):
        super().__init__(windfile_paths, [('wind_velocity', 'float64'), ('wind_angle', 'float64')])

    def _load_data(self):
        super()._load_data()
        # converts angle from geographic to trigonometric
        self._bands['wind_angle'] = geo_angle_to_trigo_angle(self._bands['wind_angle'])

    def get_wind(self, location):
        return self.get_value(location)


class WindNinjaCLI():

    def __init__(self, path=WINDNINJA_CLI_PATH, ascii_out_resolution=25, cli_arguments=None):
        self.windninja_path = path
        self.args = {}  # dict(arg, value)
        num_threads = len(os.sched_getaffinity(0)) if "sched_getaffinity" in dir(os) else 2
        self.add_arguments(num_threads=num_threads,
                           output_speed_units='kph',
                           ascii_out_resolution=int(ascii_out_resolution),
                           units_ascii_out_resolution='m',
                           units_mesh_resolution='m',
                           write_ascii_output='true')
        if cli_arguments is not None:
            if 'mesh_resolution' not in cli_arguments and 'mesh_choice' not in cli_arguments:
                # mesh_choice
                # ¡! mesh_resolution conflicts with mesh_choice
                cli_arguments['mesh_resolution'] = int(ascii_out_resolution)
            self.add_arguments(**cli_arguments)

    def args_str(self):
        return " ".join(["=".join(("".join(
            ("--", key)), value)) for key, value in self.args.items()])

    def args_list(self):
        return ["=".join(("".join(("--", key)), value)) for key, value in self.args.items()]

    @property
    def elevation_file(self):
        return self.args.get('elevation_file')

    @elevation_file.setter
    def elevation_file(self, path):
        self.args['elevation_file'] = path

    def set_elevation_file(self, file_path):
        self.elevation_file = file_path

    @property
    def output_path(self):
        return self.args.get('output_path')

    @output_path.setter
    def output_path(self, output_folder):
        """Output path.

        It must exist already. If not, WindNinja won't create it and will
        write to the default location"""
        self.args['output_path'] = output_folder

    def set_output_path(self, output_folder):
        """Set output path.

        It must exist already. If not, WindNinja won't create it and will
        write to the default location"""
        self.output_path = output_folder

    def add_arguments(self, **kwargs):
        for key, value in kwargs.items():
            self.args[str(key)] = str(value)

    def run(self):
        """Run WindNinja.

        :return: subprocess.CompletedProcess instance.
        """
        return self.run_blocking()

    def run_blocking(self):
        """Run WindNinja.

        This function blocks the execution until the process is finished.

        :return: subprocess.CompletedProcess instance.
        """
        arguments = self.args_list()
        cli = os.path.join(self.windninja_path, "WindNinja_cli")
        # Working directory must be the one in which WindNinja_cli is located
        # because it searchs locally the date_time_zonespec.csv.
        # In my opinion this is a bug in WindNinja_cli.
        logger.info("%s", "".join((cli, *arguments)))
        completed = subprocess.run((cli, *arguments), cwd=self.windninja_path)
        return completed

    @staticmethod
    def domain_average_args(input_speed, input_direction, vegetation='trees'):
        """Generate a set of arguments for WindNinja for a domain average input.

        :param input_speed: input wind speed in km/h
        :param input_direction: input wind direction in degrees
        :param vegetation: vegetation type (grass, brush or trees)
        :return: arguments for WindNinja
        :rtype: dict
        """
        assert -np.pi <= input_direction <= 2 * np.pi, "Input direction should be in radians. Received: {}".format(
            input_direction)
        args = {'initialization_method': 'domainAverageInitialization',
                'input_speed': input_speed,
                'input_speed_units': 'kph',
                'input_direction': trigo_angle_to_geo_angle(input_direction),
                'input_wind_height': '10.0',
                'units_input_wind_height': 'm',
                'output_wind_height': '10.0',
                'units_output_wind_height': 'm',
                'vegetation': vegetation}
        return args

    @staticmethod
    def diurnal_winds_args(uni_air_temp, uni_cloud_cover, date,
                           air_temp_units='C', cloud_cover_units='fraction'):
        """Generate a set of arguments to use diurnal winds in simulation.

        :param uni_air_temp: Surface air temperature.
        :param uni_cloud_cover: Cloud cover.
        :param date: datetime object defining a date, a time of the day and
            a timezone. For instance: 'Europe/Paris' or pytz.timezone('Europe/Paris')
        :param air_temp_units: Surface air temperature units {K | C | R | F}.
        :param cloud_cover_units: Cloud cover units {fraction | percent | canopy_category}.
        """
        args = {'diurnal_winds': 'true',
                'uni_air_temp': uni_air_temp,
                'air_temp_units': str(air_temp_units),
                'uni_cloud_cover': uni_cloud_cover,
                'cloud_cover_units': str(cloud_cover_units),
                'year': str(date.year),
                'month': str(date.month),
                'day': str(date.day),
                'hour': str(date.hour),
                'minute': str(date.minute),
                'time_zone': str(date.tzinfo)}
        return args

    @staticmethod
    def use_momentum_solver_args(number_of_iterations=300):
        """Generate a set of arguments for activating the momentum solver in WindNinja."""
        args = {'momentum_flag': 'true',
                'number_of_iterations': number_of_iterations}
        return args

    @staticmethod
    def output_type_args(write_ascii_output=True,
                         write_shapefile_output=False,
                         write_goog_output=False,
                         write_farsite_atm=False,
                         write_pdf_output=False,
                         write_vtk_output=False):

        args = {'write_ascii_output': str(write_ascii_output).lower(),
                'write_shapefile_output': str(write_shapefile_output).lower(),
                'write_goog_output': str(write_goog_output).lower(),
                'write_farsite_atm': str(write_farsite_atm).lower(),
                'write_pdf_output': str(write_pdf_output).lower(),
                'write_vtk_output': str(write_vtk_output).lower()}
        return args

    @staticmethod
    def weather_model_output_type_args(write_wx_model_ascii_output=True,
                                       write_wx_model_shapefile_output=False,
                                       write_wx_model_goog_output=False):
        args = {'write_wx_model_ascii_output': str(write_wx_model_ascii_output).lower(),
                'write_wx_model_shapefile_output': str(write_wx_model_shapefile_output).lower(),
                'write_wx_model_goog_output': str(write_wx_model_goog_output).lower()}
        return args
