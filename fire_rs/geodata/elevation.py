from fire_rs.geodata.basemap import DigitalMap, RasterTile


class ElevationMap(DigitalMap):
    """RGF93 Digital Elevation Map."""

    def get_elevation(self, position):
        """Get the height of a RGF93 position."""
        return self.get_value(position)


class ElevationTile(RasterTile):

    def __init__(self, filename, nodata_fill=None):
        """Initialise ElevationTile."""
        super().__init__(filename, [('elevation', 'float64')], nodata_fill)

    def get_elevation(self, location):
        """Get height of projected location."""
        return self.get_value(location)

    @property
    def z(self):
        """Return a 2D array of elevations."""
        return self.data.view('float64')

    def __getitem__(self, key):
        """Get height of projected location."""
        return self.get_elevation(key)
