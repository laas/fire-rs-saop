import numpy as np
import os
import subprocess

from affine import Affine
from datetime import datetime
from osgeo import gdal
from pytz import timezone

from environment import DigitalMap, RasterTile


class WindMap(DigitalMap):
    """Wind map."""

    def get_wind(self, position):
        """Get the wind vector of a RGF93 position."""
        return self.get_value(position)


class WindTile(RasterTile):

    def __init__(self, windvel_file, windang_file):
        """Initialise WindTile.
        :param windvel_file: wind velocity asc file path.
        :param windang_file: wind angle asc file path.
        """
        super().__init__(windvel_file)
        self.add_bands_from_file(windang_file)

        self.vel_file = windvel_file
        self.ang_file = windang_file

    def get_wind(self, location):
        return self.get_value(location)


class WindNinjaCLI():

    def __init__(self, path):
        self.windninja_path = path
        self.args = {} #  dict(arg, value)
        self.add_arguments(num_threads=len(os.sched_getaffinity(0)),
                           output_speed_units='mps',
                           mesh_resolution=100.0, # ¡! Conflicts with mesh_choice
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

    def run_blocking(self):
        """Run WindNinja.

        This function blocks the execution until the process is finished.

        Returns:
            completed: subprocess.CompletedProcess instance
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
        :return arguments for WindNinja
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


if __name__ == '__main__':
    import ipdb; ipdb.set_trace()
    cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
    cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 135))
    cli.add_arguments(**WindNinjaCLI.output_type_args(True, True, True, True, True, True))
    cli.add_arguments(**WindNinjaCLI.diurnal_winds_args(
        20, 0.25, datetime(2017, 11, 1, 12, 30, 0, 0, timezone('Europe/Paris'))))
    cli.set_elevation_file('/home/rbailonr/test_31.tif')
    cli.set_output_path('/home/rbailonr/output_path')
    completed = cli.run_blocking()
    if completed.returncode != 0:
        print("Error during execution! WindNinja returned {}.".format(completed.returncode))
