import gdal
import numpy as np
import unittest
import os

from fire_rs.geodata.elevation import ElevationMap, ElevationTile
from fire_rs.geodata.environment import DEFAULT_FIRERS_DEM_DATA


class OneIGNTileTest(unittest.TestCase):

    def setUp(self):
        self.tile = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.tif'))

    def test_raster_access(self):
        np.testing.assert_allclose(self.tile.z[0, 0], 394.67)
        for pos in [[475060.0, 6200074.0],
                    [475726.0, 6202235.0],
                    [475060.0, 6200074.0]]:

            array_coord = self.tile.projected_to_raster(pos)
            np.testing.assert_allclose(self.tile[pos],
                                       self.tile.z[array_coord[0], array_coord[1]])

    def test_raster_projected_conversion(self):
        raster_size = self.tile.raster_size
        corners = [[0, 0],
                   [raster_size[0]-1, 0],
                   [0, raster_size[1]-1],
                   [raster_size[0]-1, raster_size[1]-1]]
        for corner in corners:
            np.testing.assert_allclose(self.tile.projected_to_raster(self.tile.raster_to_projected(corner)), corner)

    def test_in_function(self):
        self.assertEqual(self.tile.raster_to_projected([0, 0]) in self.tile, True)

    def test_projected_boundaries(self):
        self.assertEqual([499987.5, 6200012.5] in self.tile, True)
        self.assertEqual([474987.5, 6200015.5] in self.tile, True)
        self.assertEqual([499987.5, 6175012.5] in self.tile, False)
        self.assertEqual([474987.5, 6175012.5] in self.tile, False)


class IGNElevationMap(unittest.TestCase):

    def setUp(self):
        self.zone1 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0475_6200_MNT_LAMB93_IGN69.tif'))
        self.zone2 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.tif'))
        self.zone3 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0475_6250_MNT_LAMB93_IGN69.tif'))
        self.zone4 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0500_6200_MNT_LAMB93_IGN69.tif'))
        self.zone5 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.tif'))
        self.zone6 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0500_6250_MNT_LAMB93_IGN69.tif'))
        self.zone7 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0525_6200_MNT_LAMB93_IGN69.tif'))
        self.zone8 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0525_6225_MNT_LAMB93_IGN69.tif'))
        self.zone9 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA, 'BDALTIV2_25M_FXX_0525_6250_MNT_LAMB93_IGN69.tif'))
        self.elevation_map = ElevationMap([self.zone1, self.zone2, self.zone3,
                                                       self.zone4, self.zone5, self.zone6,
                                                       self.zone7, self.zone8, self.zone9])

    def test_access(self):
        np.testing.assert_allclose(self.elevation_map[[474987.5, 6175012.5]],
                                   self.zone1[[474987.5, 6175012.5]])
        np.testing.assert_allclose(self.elevation_map[[485345.0, 6208062.0]],
                                   self.zone2[[485345.0, 6208062.0]])
        np.testing.assert_allclose(self.elevation_map[[497841.0, 6226454.0]],
                                   self.zone3[[497841.0, 6226454.0]])

    def test_array_extraction(self):
        elevation, (x_offset, y_offset) = self.elevation_map.get_values([[474987.5, 507900.5], [6175012.5, 6209012.5]], 25)
        z = elevation.view('float32')
        (x_size, y_size) = z.shape
        for x in range(0, x_size, 10):
            for y in range(0, y_size, 10):
                zxy = z[x, y]
                z2xy = self.elevation_map.get_value((x*25+x_offset, y*25+y_offset))
                self.assertEqual(zxy, z2xy)

if __name__ == '__main__':

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

    unittest.main()
