
from fire_rs.geodata.elevation import ElevationMap, ElevationTile

import gdal

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

zone1 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6200_MNT_LAMB93_IGN69.asc')
zone2 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.asc')
zone3 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0475_6250_MNT_LAMB93_IGN69.asc')
zone4 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6200_MNT_LAMB93_IGN69.asc')
zone5 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.asc')
zone6 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0500_6250_MNT_LAMB93_IGN69.asc')
zone7 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6200_MNT_LAMB93_IGN69.asc')
zone8 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6225_MNT_LAMB93_IGN69.asc')
zone9 = ElevationTile('/home/rbailonr/Documents/FireRS/DTM/BDALTIV2_2-0_25M_ASC_LAMB93-IGN69_D031_2016-11-17/BDALTIV2/1_DONNEES_LIVRAISON_2017-02-00098/BDALTIV2_MNT_25M_ASC_LAMB93_IGN69_D031/BDALTIV2_25M_FXX_0525_6250_MNT_LAMB93_IGN69.asc')

elevation_map = ElevationMap([zone1, zone2, zone3, zone4, zone5, zone6, zone7, zone8, zone9])

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import numpy.ma as ma

# Whole area
X = np.linspace(474987.5, 549987.5, num=100)
Y = np.linspace(6175012.5, 6250012.5, num=100)

# Smaller area
X = np.linspace(500000.0, 505000.0, num=200)
Y = np.linspace(6187500.0, 6192500.0, num=200)

Z = np.zeros((len(X), len(Y)))
for x in range(len(X)):
    for y in range(len(Y)):
        Z[x, y] = elevation_map.get_height(np.array([X[x], Y[y]]))
print(Z)
Y, X = np.meshgrid(Y, X)
fig = plt.figure()
fig2 = plt.figure()
ax2 = fig2.gca(aspect='equal')
ax = fig.gca(projection='3d')
surf = ax.plot_surface(X, Y, Z, vmin=0, vmax=1500, cmap=cm.gist_earth, linewidth=0, antialiased=False)
contour = ax2.contour(X,Y,Z, 50, cmap=cm.gist_earth)
# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)
fig2.colorbar(contour, shrink=0.5, aspect=5)
ax.auto_scale_xyz([500000.0, 505000.0], [6187500.0, 6192500.0], [0, 5000])

plt.show()
