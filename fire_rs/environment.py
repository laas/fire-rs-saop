import sys
from osgeo import gdal
import numpy as np
import numpy.ma as ma
from scipy.ndimage import affine_transform
from scipy.interpolate import RegularGridInterpolator
import logging
from affine import Affine

version_num = int(gdal.VersionInfo('VERSION_NUM'))
if version_num < 1100000:
    sys.exit('ERROR: Python bindings of GDAL 1.10 or later required')

# There are 3 sets of coordinates (spaces) for every location. A position in the geographic space, expressed in GPS
# coordinates (WGS84 system); a position in the projected space, expressed in meters north and east
# (RGF93/Lambert-93 system); and a position in the raster space, expressed as pixel/line.
# https://libraries.mit.edu/files/gis/DEM2013.pdf Slides explaining DEMs, slope calculation...
# Another projected space is UTM

class ElevationMap():
    """RGF93 Digital Elevation Map."""

    def __init__(self, zones=[]):
        """Initialise ElevationMap."""
        self._zones = []
        # self._geotransform = None

        for zone in zones:
            self.add_zone(zone)

    def add_zone(self, zone):
        """Add zone to the elevation map."""
        # if not self._geotransform:
        #     self._geotransform = zone.geotransform
        # else:
        #     if zone.geotransform != self._geotransform:
        #         logging.warning("You may have mixed incompatible map sources: zone '{}'" +
        #                         "geotransform '{}' does not match existing geotransform '{}'",
        #                         (zone, zone.geotransform, self._geotransform))
        self._zones.append(zone)

    def get_height(self, position):
        """Get height corresponding to RGF93 position."""
        for zone in self._zones:
            if position in zone:
                return zone[position]

    def __getitem__(self, key):
        """Get height corresponding to RGF93 position."""
        return self.get_height(key)


class Zone():

    FILL_VALUE = 0

    def __init__(self, filename):
        """Initialise Zone"""
        self.filehandle = gdal.Open(filename)
        self.geoprojection = self.filehandle.GetProjection()

        self.raster_size = np.array([self.filehandle.RasterXSize, self.filehandle.RasterYSize])
        self.raster_offset = np.array([0, 0])
        self.raster_bounds = [self.raster_offset, self.raster_size + self.raster_offset]

        # geotransform: coefficients for transforming between pixel/line (P,L) raster space,
        # and projection coordinates (Xp,Yp) space
        self.geotransform = self.filehandle.GetGeoTransform()

        # self.geotransform = list(self.geotransform)
        # self.geotransform[1] *= -1
        # self.geotransform[5] *= -1

        self.direct_transform = Affine.from_gdal(*self.geotransform)
        self.inverse_transform = ~self.direct_transform

        #
        #
        # # Set transformation matrices
        # # Suppose diagonal matrix (faster computations)
        # if not int(self.geotransform[2]) == 0 or not int(self.geotransform[4]) == 0:
        #     logging.warning("Affine transformation matrix is not diagonal. Resulting mapping may be wrong")
        # self.transform_matrix = np.array([[self.geotransform[1], self.geotransform[2]],
        #                                   [self.geotransform[4], self.geotransform[5]]])
        # self.proj_offset = np.array([self.geotransform[0], self.geotransform[3]])
        #
        # if self.filehandle.RasterCount > 1:
        #     logging.warning("There are {} != 1 raster bands in file {}",
        #                     self.filehandle.RasterCount, filename)
        #
        # bottom_right_corner = np.array([self.proj_offset[0] + (self.raster_size[0] * self.geotransform[1]),
        #                                 self.proj_offset[1] + (self.raster_size[1] * self.geotransform[5])])
        # self.proj_bounds = [self.proj_offset, bottom_right_corner]
        #
        topleft_projection_corner = np.array([self.direct_transform.c,
                                              self.direct_transform.f])
        bottomright_projection_corner = np.array(self.direct_transform * (self.raster_size))
        # bottomright_projection_corner = bottomright_projection_corner + np.array([
        #     self.direct_transform.a / 2, self.direct_transform.e / 2])
        self.projection_bounds = [topleft_projection_corner, bottomright_projection_corner]

        # Process band #1
        self.band = self.filehandle.GetRasterBand(1)  ## Bands start at 1
        self.metadata = self.band.GetMetadata()
        self.nodata_value = self.band.GetNoDataValue()
        arr = self.band.ReadAsArray()
        self.raster_z = ma.MaskedArray(arr, np.logical_not(np.array((arr + np.abs(self.nodata_value)),
                                                             dtype='bool')), fill_value=0.0)
        ## Regular interpolators
        # x = np.arange(self.raster_z.shape[0])
        # y = np.arange(self.raster_z.shape[0])
        # self.z_raster = RegularGridInterpolator((x, y), self.raster_z)
        #
        # x = self.proj_offset[0] + np.linspace(0, self.raster_size[0]*self.geotransform[1],
        #                                       num=np.abs(self.geotransform[0]))
        # print(x)
        # y = self.proj_offset[1] + np.linspace(0, self.raster_size[1]*self.geotransform[5],
        #                                       num=np.abs(self.geotransform[5]))
        # print(y)
        # self.z_proj = RegularGridInterpolator((x, y), self.raster_z)

    def raster_to_projected(self, loc):
        """Return the projected location of a pixel point in this zone"""
        if loc[0] in range(0, self.raster_size[0]) and loc[1] in range(0, self.raster_size[1]):
            xp, yp = self.direct_transform * (loc[0], loc[1])
            xp, yp = xp + self.direct_transform.a / 2, yp + self.direct_transform.e / 2
            return np.array([xp, yp])
        else:
            raise KeyError("Location out of this zone bounds")

    def projected_to_raster(self, loc):
        """Return the raster point of a projected location in this zone"""
        xr, yr = self.inverse_transform * (loc[0], loc[1])
        xr, yr = int(xr - 0.5), int(yr - 0.5)

        if xr in range(0, self.raster_size[0]) and yr in range(0, self.raster_size[1]):
            # TODO: Why size - xr ?!?!
            return np.array([yr, xr])
        else:
            raise KeyError("Location out of this zone bounds")


    # Slightly modified from http://stackoverflow.com/a/28238050
    def _extract_point_from_raster(point, data_source, band_number=1):
        """Return floating-point value that corresponds to given point."""

        # Convert point co-ordinates so that they are in same projection as raster
        point_sr = point.GetSpatialReference()
        raster_sr = osr.SpatialReference()
        raster_sr.ImportFromWkt(data_source.GetProjection())
        transform = osr.CoordinateTransformation(point_sr, raster_sr)
        point.Transform(transform)

        # Convert geographic co-ordinates to pixel co-ordinates
        x, y = point[0], point[1]
        forward_transform = Affine.from_gdal(*data_source.GetGeoTransform())
        reverse_transform = ~forward_transform
        px, py = reverse_transform * (x, y)
        px, py = int(px + 0.5), int(py + 0.5)

        # Extract pixel value
        band = data_source.GetRasterBand(band_number)
        structval = band.ReadRaster(px, py, 1, 1, buf_type=gdal.GDT_Float32)
        result = struct.unpack('f', structval)[0]
        if result == band.GetNoDataValue():
            result = float('nan')
        return result

    def get_height(self, location):
        """Get height of projected location."""
        p = self.projected_to_raster(location)
        return self.raster_z.filled()[p[0], p[1]]

    def __getitem__(self, key):
        """Get height of projected location."""
        return self.get_height(key)

    def __contains__(self, item):
        """Test if projected location is represented in this zone."""
        # if (self.proj_bounds[0][0] < item[0] < self.proj_bounds[1][0]) and ((self.proj_bounds[0][1] < item[1] < self.proj_bounds[1][1])):
        #     return True
        # return False
        try:
            self.projected_to_raster(item)
            return True
        except KeyError:
            return False

if __name__ == '__main__':

    # example GDAL error handler function
    def gdal_error_handler(err_class, err_num, err_msg):
        errtype = {
                gdal.CE_None:'None',
                gdal.CE_Debug:'Debug',
                gdal.CE_Warning:'Warning',
                gdal.CE_Failure:'Failure',
                gdal.CE_Fatal:'Fatal'
        }
        err_msg = err_msg.replace('\n',' ')
        err_class = errtype.get(err_class, 'None')
        print('Error Number: %s' % (err_num))
        print('Error Type: %s' % (err_class))
        print('Error Message: %s' % (err_msg))

    # install error handler
    gdal.PushErrorHandler(gdal_error_handler)

    # Enable GDAL/OGR exceptions
    gdal.UseExceptions()
    import ipdb; ipdb.set_trace()
    celda = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.asc')
    print(celda.geotransform)
    print(celda.raster_z[0, 0])
    # test corners
    # print(celda.projected_to_raster(np.array([499987.5, 6200012.5])))
    # print(celda.projected_to_raster(np.array([474987.5, 6200015.5])))
    # print(celda.projected_to_raster(np.array([499987.5, 6175012.5])))
    # print(celda.projected_to_raster(np.array([474987.5, 6175012.5])))
    import ipdb; ipdb.set_trace()
    print(celda[np.array([475060.0, 6200074.0])])

    print(celda.projected_to_raster(celda.raster_to_projected(np.array([0, 0]))))
    print(celda.projected_to_raster(celda.raster_to_projected(np.array([999, 0]))))
    print(celda.projected_to_raster(celda.raster_to_projected(np.array([0, 999]))))
    print(celda.projected_to_raster(celda.raster_to_projected(np.array([999, 999]))))

    print(celda.raster_to_projected(np.array([0, 0])) in celda)

    print(np.array([499987.5, 6200012.5]) in celda)
    print(np.array([474987.5, 6200015.5]) in celda)
    print(np.array([499987.5, 6175012.5]) in celda)
    print(np.array([474987.5, 6175012.5]) in celda)

    print(celda.projection_bounds)

    zone1 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6200_MNT_LAMB93_IGN69.asc')
    zone2 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.asc')
    zone3 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6250_MNT_LAMB93_IGN69.asc')
    zone4 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6200_MNT_LAMB93_IGN69.asc')
    zone5 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.asc')
    zone6 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6250_MNT_LAMB93_IGN69.asc')
    zone7 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6200_MNT_LAMB93_IGN69.asc')
    zone8 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6225_MNT_LAMB93_IGN69.asc')
    zone9 = Zone('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6250_MNT_LAMB93_IGN69.asc')


    import ipdb; ipdb.set_trace()
    elevation_map = ElevationMap([zone1, zone2, zone3, zone4, zone5, zone6, zone7, zone8, zone9])
    print(elevation_map[np.array([485345.0, 6208062.0])])
    print(elevation_map[np.array([497841.0, 6226454.0])])
    print(elevation_map[np.array([474987.5, 6175012.5])])

    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.ticker import LinearLocator, FormatStrFormatter
    import numpy as np
    import numpy.ma as ma


    import ipdb; ipdb.set_trace()
    X = np.linspace(474987.5, 549987.5, num=500)
    Y = np.linspace(6175012.5, 6250012.5, num=500)
    Z = np.zeros((len(X), len(Y)))
    for x in range(len(X)):
        for y in range(len(Y)):
            Z[x, y] = elevation_map.get_height(np.array([X[x], Y[y]]))
    import ipdb; ipdb.set_trace()
    print(Z)
    Y, X = np.meshgrid(Y, X)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(X, Y, Z, cmap=cm.gist_earth, linewidth=0, antialiased=False)
    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)

    plt.show()
