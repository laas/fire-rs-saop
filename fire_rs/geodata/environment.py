from decimal import Decimal
import logging
import numpy as np
import os

from elevation import ElevationMap, ElevationTile
from wind import WindMap, WindNinjaCLI, WINDNINJA_CLI_PATH


class World:
    """Class providing access to environment data."""

    def __init__(self, elevation_path, wind_path):
        self._elevation_path = os.path.abspath(elevation_path)
        self._elevation_map = ElevationMap([])
        self._load_elevation_tiles()

        self._wind_path = os.path.abspath(wind_path)
        self._wind_maps = {'domainAverageInitialization': {},
                           'pointInitialization': {},
                           'wxModelInitialization': {}}
        # Wind maps is a collection of wind scenarios, sorted by initialization method.
        # For domainAverageInitialization maps are sorted by (speed, direction)

        domain_scenario = WindNinjaCLI(WINDNINJA_CLI_PATH)
        domain_scenario.add_arguments(**WindNinjaCLI.domain_average_args(0,0))
        domain_scenario.add_arguments(**WindNinjaCLI.output_type_args())
        domain_scenario.set_output_path(self._wind_path)
        self._windninja_domain = domain_scenario

    def _load_elevation_tiles(self):
        for f in os.scandir(self._elevation_path):
            if f.is_file():
                try:
                    tile = ElevationTile(f.path)
                    self._elevation_map.add_tile(tile)
                except RuntimeError as e:
                    logging.warning(e.msg)

    def get_elevation(self, position):
        return self._elevation_map.get_elevation(position)

    def get_wind(self, position, **kwargs):
        """Get wind for a specific wind scenario.

        keyword arguments:
            domain_average: (speed, direction)
                speed is rounded to the units
                direction is rounded to degree precision
        """
        dom_av = kwargs.get('domain_average')
        if dom_av is None:
            raise TypeError("Only domainAverageInitialization method is implemented.")
        else:
            # Detect if a map for this wind case has been loaded, and create if if not
            dom_av = ("{:.0f}".format(dom_av[0]), "{:.0f}".format(dom_av[1]))
            wind_map = self._wind_maps['domainAverageInitialization'].get(dom_av)
            if wind_map is None:
                windninja = WindNinjaCLI(WINDNINJA_CLI_PATH, self._windninja_domain.args)
                windninja.add_arguments(**{'input_speed':dom_av[0], 'input_direction': dom_av[1]})
                m = WindMap([], self._elevation_map, windninja)
                self._wind_maps['domainAverageInitialization'][dom_av] = m
                return self._wind_maps['domainAverageInitialization'][dom_av].get_wind(position)
            return wind_map.get_wind(position)
