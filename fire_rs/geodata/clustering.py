import numpy as np
from matplotlib import pyplot
from sklearn.cluster import KMeans
from fire_rs.geodata.environment import World


area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
area_wind = (10, np.pi)

world = World()
some_area_wind = world.get_wind(area, domain_average=area_wind)

some_area_wind.plot_vector(dir_layer='wind_angle', length_layer='wind_velocity', downscale=10)

elev = world.get_elevation(area)

# need to reshape so that the second dimension is the size of the feature list (1 in our case)
X = elev.data.view('float32').reshape((elev.data.size, 1))
kmeans = KMeans(n_clusters=10, random_state=0, max_iter=300).fit(X)
clustered = kmeans.labels_.reshape(elev.data.shape)
c = elev.clone(data_array=clustered, dtype=[('cluster', 'int32')])

c.plot('cluster')
elev.plot()


pyplot.show()  # blocking until all plot windows are closed
