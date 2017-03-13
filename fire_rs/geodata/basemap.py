from affine import Affine
from collections import Sequence
import logging
import numpy as np
from osgeo import gdal


# There are 3 sets of coordinates (spaces) for every location. A position in the geographic space, expressed in GPS
# coordinates (WGS84 system); a position in the projected space, expressed in meters north and east
# (RGF93/Lambert-93 system); and a position in the raster space, expressed as pixel/line.
# https://libraries.mit.edu/files/gis/DEM2013.pdf Slides explaining DEMs, slope calculation...
# Another projected space is UTM

class DigitalMap():
    """Abstract representation of a set of tiles."""

    def __init__(self, tiles):
        """Initialise DigitalMap."""
        self._tiles = []

        for tile in tiles:
            self.add_tile(tile)

    def add_tile(self, tile):
        """Add a tile to the map."""
        self._tiles.append(tile)

    def get_value(self, position):
        """Get the value corresponding to a position."""
        # TODO: Improve performance
        for tile in self._tiles:
            if position in tile:
                return tile[position]

    def __getitem__(self, key):
        """Get the value corresponding to a position."""
        return self.get_value(key)

    def __contains__(self, position):
        """Whether a position is defined in the map."""
        # TODO: Improve performance
        for tile in self._tiles:
            if position in tile:
                return True
        return False

    def tile_of_location(self, position):
        """Get the tile where the position is defined."""
        for tile in self._tiles:
            if position in tile:
                return tile
        raise KeyError("Location {} not in {}".format(location, self))


class RasterTile():

    def __init__(self, filenames, nodata_fill=None):
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
        topleft_projection_corner = np.array([self.direct_transform.c, self.direct_transform.f])
        bottomright_projection_corner = np.array(self.direct_transform * (self.raster_size))
        self.projection_bounds = [topleft_projection_corner, bottomright_projection_corner]
        n_bands += handle.RasterCount
        for i in range(handle.RasterCount):  # Bands start at 1
            nodata_values.append(handle.GetRasterBand(i + 1).GetNoDataValue())

        # Process the rest of files
        for f in self.filenames[1:]:
            handle = gdal.Open(f)
            if handle.GetProjection() == self.geoprojection and \
               handle.GetGeoTransform() == self.geotransform:
                n_bands+=handle.RasterCount
                for i in range(handle.RasterCount):  # Bands start at 1
                    nodata_values.append(handle.GetRasterBand(i + 1).GetNoDataValue())
            else:
                raise ValueError("Only bands with the same projection can be added.")


        # Set data array size
        self._bands = np.empty((*self.raster_size, n_bands))
        self._loaded = False
        self.nodata_values = np.array(nodata_values)

    def _load_data(self):
        """Load data from files."""
        for i in range(len(self.filenames)):
            handle = gdal.Open(self.filenames[i])
            for j in range(handle.RasterCount):  # Bands start at 1
                self._bands[..., i + j] = handle.GetRasterBand(j + 1).ReadAsArray()
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
        nd = np.isclose(value, self.nodata_values)
        if nd.any():
            if not self.nodata_fill:
                # No replacement for NODATA
                logging.warning("Location {} in {} has NODATA but no fill has been defined",
                                (p[0], p[1]), self)
                return value
            else:
                for i in range(len(nd)):
                    if nd:
                        logging.warning("Location {} of band {} in {} with {}",
                            (p[0], p[1]), i, self, self.nodata_fill[i])
                        value[i] = self.nodata_fill[i]
        return value

    def raster_to_projected(self, loc):
        """Return the projected location of a pixel point in this tile"""
        if loc[0] in range(0, self.raster_size[0]) and loc[1] in range(0, self.raster_size[1]):
            xp, yp = self.direct_transform * (loc[0], loc[1])
            xp, yp = xp + self.direct_transform.a / 2, yp + self.direct_transform.e / 2
            return np.array([xp, yp])
        else:
            raise KeyError("Location out of this tile bounds")

    def projected_to_raster(self, loc):
        """Return the raster point of a projected location in this tile"""
        xr, yr = self.inverse_transform * (loc[0], loc[1])
        xr, yr = int(xr - 0.5), int(yr - 0.5)

        if xr in range(0, self.raster_size[0]) and yr in range(0, self.raster_size[1]):
            return np.array([xr, yr])
        else:
            raise KeyError("Location out of this tile bounds")
