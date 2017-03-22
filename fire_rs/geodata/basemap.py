import functools
import logging
from collections import Sequence

import numpy as np
from affine import Affine
from osgeo import gdal

from fire_rs.geodata.geo_data import GeoData, join_structured_arrays


# There are 3 sets of coordinates (spaces) for every location. A position in the geographic space,
# expressed in GPS coordinates (WGS84 system); a position in the projected space, expressed in
# meters north and east (RGF93/Lambert-93 system); and a position in the raster space, expressed
# as pixel/line.
# https://libraries.mit.edu/files/gis/DEM2013.pdf Slides explaining DEMs, slope calculation...
# Another projected space is UTM

class DigitalMap:
    """Abstract representation of a set of tiles."""

    def __init__(self, tiles):
        """Initialise DigitalMap. Tiles should entirely cover a rectangular area."""
        self._tiles = [[]]  # a 2D-array of tiles, covering a rectangular area
        for tile in tiles:
            self.add_tile(tile)

    def add_tile(self, tile):
        """Add a tile to the map."""
        # gather all tiles and sort them by increasing (x, y)
        all_tiles = [t for ts in self._tiles for t in ts] + [tile]
        all_tiles.sort(key=lambda t: (t.x_min, t.y_min))

        # repopulate tiles in a sorted 2D array
        self._tiles = [[]]
        prev_x_min = None
        for t in all_tiles:
            if prev_x_min is not None and prev_x_min != t.x_min:
                self._tiles.append([t])  # changed x origin, add new line
            else:
                self._tiles[-1].append(t)  # append to current line
            prev_x_min = t.x_min

    def get_value(self, position):
        """Get the value corresponding to a position."""
        for horizontal_tiles in self._tiles:
            for tile in horizontal_tiles:
                if position in tile:
                    return tile[position]

    def get_values(self, positions_intervals, cell_size):
        assert cell_size == abs(self._tiles[0][0].x_delta) and \
            cell_size == abs(self._tiles[0][0].y_delta)
        ((x_min, x_max), (y_min, y_max)) = positions_intervals
        xi_min = xi_max = yi_min = yi_max = -1
        for xi, txs in enumerate(self._tiles):
            for yi, t in enumerate(txs):
                if [x_min, y_min] in t:
                    xi_min, yi_min = xi, yi
                if [x_max, y_max] in t:
                    xi_max, yi_max = xi, yi

        assert xi_min != -1 and yi_min != -1, "({}, {}) is out of bounds of the known tiles".format(
            x_min, y_min)
        assert xi_max != -1 and yi_max != -1, "({}, {}) is out of bounds of the known tiles".format(
            x_max, y_max)
        assert xi_min <= xi_max and yi_min <= yi_max

        # round coordinates to cell centers
        x_min, y_min = self._tiles[xi_min][yi_min].nearest_projected_point((x_min, y_min))
        x_max, y_max = self._tiles[xi_max][yi_max].nearest_projected_point((x_max, y_max))

        current_y = y_min
        tables = []
        for yi in range(yi_min, yi_max + 1):
            tables.append([])  # add new line
            current_x = x_min
            for xi in range(xi_min, xi_max+1):
                tile = self._tiles[xi][yi]
                next_x = min(x_max, tile.x_max)
                next_y = min(y_max, tile.y_max)
                table = tile.get_values(((current_x, next_x), (current_y, next_y)))
                tables[-1].append(table)  # append table to last line
                current_x = next_x + cell_size
            current_y = next_y + cell_size

        # concatenates tables on the x-axis and then the resulting tables on the y-axis
        combined_xs = [functools.reduce(GeoData.append_right, ts) for ts in tables]
        result = functools.reduce(GeoData.append_bottom, combined_xs)
        return result, (x_min, y_min)

    def __getitem__(self, key):
        """Get the value corresponding to a position."""
        return self.get_value(key)

    def __contains__(self, position):
        """Whether a position is defined in the map."""
        for horizontal_tiles in self._tiles:
            for tile in horizontal_tiles:
                if position in tile:
                    return True
        return False

    def tile_of_location(self, position):
        """Get the tile where the position is defined."""
        for horizontal_tiles in self._tiles:
            for tile in horizontal_tiles:
                if position in tile:
                    return tile
        raise KeyError("Location {} not in {}".format(position, self))


class RasterTile:

    def __init__(self, filenames, bands_names_types, nodata_fill=None):
        """Initialise RasterTile.

        It stores the metadata during the initilization. Data loading is lazy.

        :param filenames: One or more file names corresponding to the same tile.
        :param nodata_fill: Array of of size equal to the total number of bands
        """
        if isinstance(filenames, str):
            filenames = [filenames]
        elif isinstance(filenames, Sequence):
            if len(filenames) < 1:
                raise ValueError("filenames is empty")
        else:
            raise TypeError("filenames must be a string or sequence of strings")

        self.filenames = filenames
        self.nodata_fill = nodata_fill
        self.bands_names_types = bands_names_types

        n_bands = 0
        nodata_values = []

        # First file
        handle = gdal.Open(filenames[0])
        self.geoprojection = handle.GetProjection()
        self.raster_size = np.array([handle.RasterXSize, handle.RasterYSize])
        self.raster_offset = np.array([0, 0])
        self.raster_bounds = [self.raster_offset, self.raster_size + self.raster_offset]

        # geotransform: coefficients for transforming between pixel/line (P,L) raster space,
        # and projection coordinates (Xp,Yp) space
        self.geotransform = handle.GetGeoTransform()
        self.direct_transform = Affine.from_gdal(*self.geotransform)
        self.inverse_transform = ~self.direct_transform
        topleft_projection_corner = (self.direct_transform.c, self.direct_transform.f)
        bottomright_projection_corner = self.direct_transform * (self.raster_size)
        self.projection_bounds = [topleft_projection_corner, bottomright_projection_corner]

        # min/max values of the the projected cell centers
        self.x_min, self.y_min = self.nearest_projected_point((topleft_projection_corner[0]+1,
                                                               bottomright_projection_corner[1]+1))
        self.x_max, self.y_max = self.nearest_projected_point((bottomright_projection_corner[0]-1,
                                                               topleft_projection_corner[1]-1))

        # delta between two cell centers (can be negative)
        self.x_delta = self.geotransform[1]
        self.y_delta = self.geotransform[5]

        n_bands += handle.RasterCount
        for i in range(handle.RasterCount):  # Bands start at 1
            nodata_values.append(handle.GetRasterBand(i + 1).GetNoDataValue())

        # Process the rest of files
        for f in self.filenames[1:]:
            handle = gdal.Open(f)
            if handle.GetProjection() == self.geoprojection and \
               handle.GetGeoTransform() == self.geotransform:
                n_bands += handle.RasterCount
                for i in range(handle.RasterCount):  # Bands start at 1
                    nodata_values.append(handle.GetRasterBand(i + 1).GetNoDataValue())
            else:
                raise ValueError("Only bands with the same projection can be added.")

        assert n_bands == len(bands_names_types),\
            "Number of bands in files ({}) did not match the declared ones: {}".format(
                n_bands, bands_names_types)

        # Set data array size
        self._bands = None
        self._loaded = False
        self.nodata_values = np.array(nodata_values)

    def _load_data(self):
        """Load data from files."""
        curr_layer = 0  # tracks the layer we are currently looking at
        layers = []  # a sequence of structured arrays: one by layer
        for i in range(len(self.filenames)):
            handle = gdal.Open(self.filenames[i])
            for j in range(handle.RasterCount):  # Bands start at 1
                layer = np.array(handle.GetRasterBand(j + 1).ReadAsArray(),
                                 dtype=[self.bands_names_types[curr_layer]])
                # make x the first index and y the second, keeping the C ordering
                layer = np.array(layer.transpose(), order='C')
                layers.append(layer)
                curr_layer += 1

        assert curr_layer == len(self.bands_names_types), "Less layers that expected"
        assert all(l.shape == layers[0].shape for l in layers), "Layers do not have a uniform shape"
        self._bands = join_structured_arrays(layers)
        self._loaded = True

    @property
    def data(self):
        if not self._loaded:
            self._load_data()
        return self._bands

    @property
    def loaded(self):
        return self._loaded

    def __getitem__(self, key):
        """Get height of projected location."""
        return self.get_value(key)

    def __contains__(self, item):
        """Test if projected location is represented in this tile."""
        # TODO: Implement a faster way
        try:
            self.projected_to_raster(item)
            return True
        except KeyError:
            return False

    def get_value(self, location):
        """Get value of projected location."""
        p = self.projected_to_raster(location)
        value = self.data[p[0], p[1]]

        # NODATA test
        nd = np.isclose(value.tolist(), self.nodata_values)
        if nd.any():
            if not self.nodata_fill:
                # No replacement for NODATA
                logging.warning("Location %s in %s has NODATA but no fill has been defined",
                                (p[0], p[1]), self)
            else:
                for i in range(len(nd)):
                    if nd:
                        logging.warning("Location %s of band %s in %s with %s",
                                        (p[0], p[1]), i, self, self.nodata_fill[i])
                        value[i] = self.nodata_fill[i]
        if len(value) == 1:
            return value[0]  # Extract if there is a single value
        else:
            return value

    def as_geo_data(self):
        return self.get_values(((self.x_min, self.x_max), (self.y_min, self.y_max)))

    def get_values(self, rectangle):
        """Return an array covering the given rectangle.

        Increasing x/y array indexes correspond to increasing value in the projected space.
        Cell size is the one of the tile.
        """
        assert self.x_delta > 0
        ((x_min, x_max), (y_min, y_max)) = rectangle
        if self.y_delta > 0:
            # get indexes of sub-array
            xi_min, yi_min = self.projected_to_raster((x_min, y_min))
            xi_max, yi_max = self.projected_to_raster((x_max, y_max))
            # returns the subarray
            subarray = self.data[xi_min:xi_max+1, yi_min:yi_max+1]
            return GeoData(subarray, *self.raster_to_projected((xi_min, yi_min)),
                           self.x_delta, self.y_delta)
        else:  # our internal data structure is inversed on y-axis
            # get indexes of sub-array
            xi_min, yi_min = self.projected_to_raster((x_min, y_max))
            xi_max, yi_max = self.projected_to_raster((x_max, y_min))
            # returns the sub-array, inversed on the y-axis
            subarray = self.data[xi_min:xi_max+1, yi_min:yi_max+1][..., ::-1]
            return GeoData(subarray, *self.raster_to_projected((xi_min, yi_max)),
                           self.x_delta, -self.y_delta)

    def raster_to_projected(self, loc):
        """Return the projected location of a pixel point in this tile."""
        if loc[0] in range(0, self.raster_size[0]) and loc[1] in range(0, self.raster_size[1]):
            xp, yp = self.direct_transform * (loc[0], loc[1])
            xp, yp = xp + self.direct_transform.a / 2, yp + self.direct_transform.e / 2
            return np.array([xp, yp])
        else:
            raise KeyError("Location out of this tile bounds")

    def projected_to_raster(self, loc):
        """Return the raster point of a projected location in this tile."""
        xr, yr = self.inverse_transform * (loc[0], loc[1])
        xr, yr = int(xr - 0.5), int(yr - 0.5)

        if xr in range(0, self.raster_size[0]) and yr in range(0, self.raster_size[1]):
            return np.array([xr, yr])
        else:
            raise KeyError("Location out of this tile bounds")

    def nearest_projected_point(self, loc):
        """Return the (projected) coordinates of the cell center nearest to loc."""
        return self.raster_to_projected(self.projected_to_raster(loc))
