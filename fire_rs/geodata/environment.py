import logging
import os
import numbers
import numpy as np

from fire_rs.geodata.elevation import ElevationMap, ElevationTile
from fire_rs.geodata.wind import WindMap, WindNinjaCLI
from fire_rs.geodata.geo_data import GeoData

# Set default data folders as the one given in the environment variable $FIRERS_DATA or
# to Rafael's home if not set
# TODO: check whether it is useful/desirable to keep the internal folder
DEFAULT_FIRERS_DATA_FOLDER = os.environ['FIRERS_DATA'] \
    if 'FIRERS_DATA' in os.environ else '/home/rbailonr/firers_data'
DEFAULT_FIRERS_DEM_DATA = os.path.join(DEFAULT_FIRERS_DATA_FOLDER,
                                       'dem/BDALTIV2_2-0_25M_TIF_LAMB93-IGN69_D031_2016-11-17')
DEFAULT_FIRERS_WIND_DATA = os.path.join(DEFAULT_FIRERS_DATA_FOLDER,
                                        'wind/BDALTIV2_2-0_25M_TIF_LAMB93-IGN69_D031_2016-11-17')


class World:
    """Class providing access to environment data."""

    def __init__(self, elevation_path=DEFAULT_FIRERS_DEM_DATA, wind_path=DEFAULT_FIRERS_WIND_DATA):
        self._elevation_path = os.path.abspath(elevation_path)
        self._elevation_map = ElevationMap([])
        self._load_elevation_tiles()

        self._wind_path = os.path.abspath(wind_path)
        self._wind_maps = {'domainAverageInitialization': {},
                           'pointInitialization': {},
                           'wxModelInitialization': {}}
        # Wind maps is a collection of wind scenarios, sorted by initialization method.
        # For domainAverageInitialization maps are sorted by (speed, direction)

        domain_scenario = WindNinjaCLI()
        domain_scenario.add_arguments(**WindNinjaCLI.domain_average_args(0, 0))
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

    def get_elevation(self, position) -> 'GeoData':
        """Retrieves elevation of a given point/area of the map.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]
        """
        assert len(position) == 2, "Need coordinates for x and y"

        if isinstance(position[0], numbers.Number) and isinstance(position[1], numbers.Number):  # point
            return self._elevation_map.get_elevation(position)
        else:  # position is a rectangle
            assert len(position[0]) == 2 and len(position[1]) == 2
            return self._elevation_map.get_values(position)

    def get_slope(self, area) -> 'GeoData':
        """Returns a GeoData containing the slope percentage and raise direction of an area."""
        ((x_min, x_max), (y_min, y_max)) = area

        # extract DEM on a slightly large area to avoid border effects
        dem = self.get_elevation([[x_min-25, x_max+25], [y_min-25, y_max+25]])
        z = dem.data.view(np.float32)
        assert dem.data.shape == z.shape, 'Apparently, the returned DEM is not an array of float'

        def rolled(x_roll, y_roll):
            """Returns a view of the DEM array rolled on X/Y axis"""
            return np.roll(np.roll(z, x_roll, axis=0), y_roll, axis=1)

        # compute elevation change on x and y direction, cf:
        # http://desktop.arcgis.com/fr/arcmap/10.3/tools/spatial-analyst-toolbox/how-slope-works.htm
        dzdx = rolled(-1, -1) + 2*rolled(-1, 0) + rolled(-1, 1) - rolled(1, -1) - 2*rolled(1, 0) - rolled(1, -1)
        dzdx /= (8 * dem.cell_width)
        dzdy = rolled(1, 1) + 2*rolled(0, 1) + rolled(-1, 1) - rolled(1, -1) - 2*rolled(0, -1) - rolled(-1, -1)
        dzdy /= (8 * dem.cell_width)

        # get percentage of slope and the direction of raise and save them as GeoData
        slope_percent = np.sqrt(np.power(dzdx, 2) + np.power(dzdy, 2)) * 100
        raise_dir = np.arctan2(dzdy, dzdx)
        sp = dem.clone(np.array(slope_percent, dtype=[('slope', 'float32')]))
        rd = dem.clone(np.array(raise_dir, dtype=[('raise_dir', 'float32')]))

        # combine slope and raise direction into one GeoData and fit it to the area originally asked
        return sp.combine(rd).subset(area)

    def get_wind(self, position, **kwargs) -> 'GeoData':
        """Get wind for a specific wind scenario.
        Position is either a point [x, y] or an area [[x_min, x_max], [y_min, y_max]]

        keyword arguments:
            domain_average: (speed, direction)
                speed is rounded to the units
                direction is rounded to degree precision
        """
        assert len(position) == 2, "Need coordinates for x and y"
        dom_av = kwargs.get('domain_average')
        if dom_av is None:
            raise TypeError("Only domainAverageInitialization method is implemented.")
        else:
            from .wind import trigo_angle_to_geo_angle
            assert -np.pi <= dom_av[1] <= 2*np.pi, \
                "Wind direction should be given in radians, where 0 is East to West and rotation is trigonometric"
            # Detect if a map for this wind case has been loaded, and create if not
            dom_av = ("{:.0f}".format(dom_av[0]), "{:.0f}".format(trigo_angle_to_geo_angle(dom_av[1])))
            wind_map = self._wind_maps['domainAverageInitialization'].get(dom_av)

            if wind_map is None:  # create a new wind map for this domain average
                windninja = WindNinjaCLI(cli_arguments=self._windninja_domain.args)
                windninja.add_arguments(**{'input_speed':dom_av[0], 'input_direction': dom_av[1]})
                wind_map = WindMap([], self._elevation_map, windninja)
                self._wind_maps['domainAverageInitialization'][dom_av] = wind_map

            if isinstance(position[0], numbers.Number) and isinstance(position[1], numbers.Number):  # point
                return wind_map.get_wind(position)
            else:  # position is a rectangle
                assert len(position[0]) == 2 and len(position[1]) == 2
                return wind_map.get_values(position)
