import random
import numpy as np
import scipy.spatial
import scipy.spatial.distance
import fire_rs.planning.uav as uav
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

uav_home_position = (0,0)

grid_size = (1000, 1000)
n_poi = 50
points_of_interest = np.empty((n_poi, 2))

points_of_interest[0:20]   = np.array([(random.randint(0, 50), random.randint(0, 50)) for i in range(20)])
points_of_interest[20:40] = np.array([(random.randint(50, 100), random.randint(50, 100)) for i in range(20)])
points_of_interest[40:45] = np.array([(random.randint(0, 50), random.randint(50, 100)) for i in range(5)])
points_of_interest[45:50] = np.array([(random.randint(50, 100), random.randint(0, 50)) for i in range(5)])
poi_radius = 1  # Distance from wich the poi is considered as visited

def density_weight(all_nodes):
    '''Calculate the relevance weight of each node.
    
    For every node, its weight is the sum of the inverse of the squared eculidean distance to any other node.
    '''
    # TODO: the distance to the UAV should be taken in account, so the closest cumulus of points
    # get more importance than those that are further.
    distance_matrix = scipy.spatial.distance.cdist(all_nodes, all_nodes, 'euclidean')
    for i in range(len(distance_matrix)):
        distance_matrix[i,i] = np.inf

    weights = np.sum(1/distance_matrix**2, axis=0)
    max_weight = np.max(weights)
    return weights /max_weight

poi_weights = density_weight(points_of_interest)

tesselation = scipy.spatial.Delaunay(points_of_interest)

fig = plt.figure()
ax1 = fig.add_subplot(121, aspect='equal')

plt.triplot(points_of_interest[:,0], points_of_interest[:,1], tesselation.simplices.copy())
plt.scatter(points_of_interest[:,0], points_of_interest[:,1], poi_weights*100, c='red')

# ax2 = fig.add_subplot(122, aspect='equal')
# plt.scatter(points_of_interest[:,0], points_of_interest[:,1], poi_weights*10)

plt.show()
print()
