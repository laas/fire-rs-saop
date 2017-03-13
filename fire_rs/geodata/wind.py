from datetime import datetime
import os
from pytz import timezone
import subprocess

from basemap import DigitalMap, RasterTile


WINDNINJA_CLI_PATH = '/home/rbailonr/bin/windninja_cli'
if not os.path.exists(os.path.join(WINDNINJA_CLI_PATH, 'WindNinja_cli')):
    raise FileNotFoundError("WINDNINJA_CLI_PATH is not defined correctly")


class WindMap(DigitalMap):
    """Wind map asociated to an elevation map."""

    def __init__(self, tiles, elevation_map, windninja):
        """Initialise WindMap.

        :param tiles: Sequence of WindTiles
        :param elevation_map: Associated ElevationMap.
        :param scenario: WindNinjaCLI instance with all the necessary arguments set.
        """
        super().__init__(tiles)
        self.elevation_map = elevation_map

        self.scenario = windninja.args
        self.windninja_cli = windninja

        sce_list = []
        if self.scenario['initialization_method'] == 'domainAverageInitialization':
            sce_list.append(str(int(float(self.scenario['input_direction']))))
            sce_list.append(str(int(float(self.scenario['input_speed']))))
            if bool(self.scenario['diurnal_winds']):
                sce_list.append('-'.join([self.scenario['month'],
                                          self.scenario['day'],
                                          self.scenario['year']]))
                sce_list.append(''.join([self.scenario['hour'],
                                         self.scenario['minute']]))

            sce_list.append(''.join([str(self.scenario['mesh_resolution']),
                                    self.scenario['units_mesh_resolution']]))

        self.scenario_str = '_'.join(sce_list)  # part of WindNinja output file(s)

    def _load_tile(self, base_tile):
        tile_name = os.path.splitext(os.path.split(base_tile.filenames[0])[1])[0]
        windfile_paths = [os.path.join(self.scenario['output_path'],
                                       '_'.join([tile_name,
                                                 self.scenario_str,
                                                 'vel.asc'])),
                          os.path.join(self.scenario['output_path'],
                                       '_'.join([tile_name,
                                                 self.scenario_str,
                                                 'ang.asc']))]

        if not(os.path.exists(windfile_paths[0]) and os.path.exists(windfile_paths[1])):
            self._run_windninja(base_tile)

        return WindTile(windfile_paths)

    def _run_windninja(self, elevation_file):
        self.windninja_cli.set_elevation_file(elevation_file.filenames[0])
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
            tile = self._load_tile(self.elevation_map.tile_of_location(position))
            self.add_tile(tile)
            return tile[position]
        return super().get_value()

    def get_wind(self, position):
        """Get the wind vector of a position."""
        return self.get_value(position)


class WindTile(RasterTile):

    def get_wind(self, location):
        return self.get_value(location)


class WindNinjaCLI():

    def __init__(self, path):
        self.windninja_path = path
        self.args = {} #  dict(arg, value)
        self.add_arguments(num_threads=len(os.sched_getaffinity(0)),
                           output_speed_units='mps',
                           mesh_resolution=100, # ¡! Conflicts with mesh_choice
                           units_mesh_resolution='m',
                           write_ascii_output='true')

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
        completed = subprocess.run((cli, *arguments), cwd=self.windninja_path)
        return completed

    @staticmethod
    def domain_average_args(input_speed, input_direction, vegetation='trees'):
        """Generate a set of arguments for WindNinja for a domain average input.

        :param input_speed: input wind speed in m/s
        :param input_direction: input wind direction in degrees
        :param vegetation: vegetation type (grass, brush or trees)
        :return: arguments for WindNinja
        :rtype: dict
        """
        args = {'initialization_method'   : 'domainAverageInitialization',
                'input_speed'             : input_speed,
                'input_speed_units'       : 'mps',
                'input_direction'         : input_direction,
                'input_wind_height'       : '10.0',
                'units_input_wind_height' : 'm',
                'output_wind_height'      : '10.0',
                'units_output_wind_height': 'm',
                'vegetation'              : vegetation}
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
        args = {'diurnal_winds'    : 'true',
                'uni_air_temp'     : uni_air_temp,
                'air_temp_units'   : str(air_temp_units),
                'uni_cloud_cover'  : uni_cloud_cover,
                'cloud_cover_units': str(cloud_cover_units),
                'year'             : str(date.year),
                'month'            : str(date.month),
                'day'              : str(date.day),
                'hour'             : str(date.hour),
                'minute'           : str(date.minute),
                'time_zone'        : str(date.tzinfo)}
        return args

    @staticmethod
    def use_momentum_solver_args(number_of_iterations=300):
        """Generate a set of arguments for activating the momentum solver in WindNinja."""
        args = {'momentum_flag'       : 'true',
                'number_of_iterations': number_of_iterations}
        return args

    @staticmethod
    def output_type_args(write_ascii_output=True,
                         write_shapefile_output=False,
                         write_goog_output=False,
                         write_farsite_atm=False,
                         write_pdf_output=False,
                         write_vtk_output=False):

        args = {'write_ascii_output'    : str(write_ascii_output).lower(),
                'write_shapefile_output': str(write_shapefile_output).lower(),
                'write_goog_output'     : str(write_goog_output).lower(),
                'write_farsite_atm'     : str(write_farsite_atm).lower(),
                'write_pdf_output'      : str(write_pdf_output).lower(),
                'write_vtk_output'      : str(write_vtk_output).lower()}
        return args

    @staticmethod
    def weather_model_output_type_args(write_wx_model_ascii_output=True,
                                       write_wx_model_shapefile_output=False,
                                       write_wx_model_goog_output=False):
        args = {'write_wx_model_ascii_output'    : str(write_wx_model_ascii_output).lower(),
                'write_wx_model_shapefile_output': str(write_wx_model_shapefile_output).lower(),
                'write_wx_model_goog_output'     : str(write_wx_model_goog_output).lower()}
        return args
