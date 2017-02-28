import numpy as np
import gdal

import unittest

import environment


class OneIGNTileTest(unittest.TestCase):

    def setUp(self):
        self.tile = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.asc')

    def test_raster_access(self):
        np.testing.assert_allclose(self.tile.raster_z[0, 0], 394.67)
        for pos in [np.array([475060.0, 6200074.0]),
                    np.array([475726.0, 6202235.0]),
                    np.array([475060.0, 6200074.0])]:

            array_coord = self.tile.projected_to_raster(pos)
        np.testing.assert_allclose(self.tile[pos],
                                   self.tile.raster_z[array_coord[0], array_coord[1]])

    def test_raster_projected_conversion(self):
        raster_size = self.tile.raster_size
        corners = [np.array([0, 0]),
                   np.array([raster_size[0]-1, 0]),
                   np.array([0, raster_size[1]-1]),
                   np.array([raster_size[0]-1, raster_size[1]-1])]
        for corner in corners:
            np.testing.assert_allclose(self.tile.projected_to_raster(self.tile.raster_to_projected(corner)), corner)

    def test_in_function(self):
        self.assertEqual(self.tile.raster_to_projected(np.array([0, 0])) in self.tile, True)

    def test_projected_boundaries(self):
        self.assertEqual(np.array([499987.5, 6200012.5]) in self.tile, True)
        self.assertEqual(np.array([474987.5, 6200015.5]) in self.tile, True)
        self.assertEqual(np.array([499987.5, 6175012.5]) in self.tile, False)
        self.assertEqual(np.array([474987.5, 6175012.5]) in self.tile, False)


class IGNElevationMap(unittest.TestCase):

    def setUp(self):
        self.zone1 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6200_MNT_LAMB93_IGN69.asc')
        self.zone2 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.asc')
        self.zone3 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6250_MNT_LAMB93_IGN69.asc')
        self.zone4 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6200_MNT_LAMB93_IGN69.asc')
        self.zone5 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.asc')
        self.zone6 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6250_MNT_LAMB93_IGN69.asc')
        self.zone7 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6200_MNT_LAMB93_IGN69.asc')
        self.zone8 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6225_MNT_LAMB93_IGN69.asc')
        self.zone9 = environment.MapTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6250_MNT_LAMB93_IGN69.asc')
        self.elevation_map = environment.ElevationMap([self.zone1, self.zone2, self.zone3,
                                                       self.zone4, self.zone5, self.zone6,
                                                       self.zone7, self.zone8, self.zone9])

    def test_access(self):
        np.testing.assert_allclose(self.elevation_map[np.array([474987.5, 6175012.5])],
                                   self.zone1[np.array([474987.5, 6175012.5])])
        np.testing.assert_allclose(self.elevation_map[np.array([485345.0, 6208062.0])],
                                   self.zone2[np.array([485345.0, 6208062.0])])
        np.testing.assert_allclose(self.elevation_map[np.array([497841.0, 6226454.0])],
                                   self.zone3[np.array([497841.0, 6226454.0])])


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
