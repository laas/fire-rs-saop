from fire_rs.geodata.basemap import DigitalMap, RasterTile


class LandCoverMap(DigitalMap):
    """Land Cover Map."""

    def get_class(self, position):
        """Get the land cover class of a RGF93 position."""
        return self.get_value(position)


class LandCoverTile(RasterTile):

    def __init__(self, filename, nodata_fill=None):
        """Initialise LandCoverTile."""
        super().__init__(filename, [('landcover', 'uint8')], nodata_fill)

    def get_class(self, location):
        """Get the land cover class of projected location."""
        return self.get_value(location)

    def __getitem__(self, key):
        """Getthe the land cover class of projected location."""
        return self.get_value(key)
