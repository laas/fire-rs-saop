import numpy as np
from matplotlib import pyplot
from fire_rs.geodata.environment import World
from fire_rs.geodata.geo_data import GeoData


def cluster_multi_layer(arr: 'GeoData', err_by_layer, already_clustered=(), cluster_layer_name='clustering'):
    """Clusters a GeoData by (1) clustering each of its layers individually, (2) combining each clustered layer 
    into a new clustering such that two cells are in the same cluster all the each of their individual layers are in 
    the same cluster.
    
    Arguments: 
        - err_by_layer: maps a layer to the maximum distance between two points of the cluster
         For instance: {'l1': 2, ...}, two points of the layer 'l1' can be the same cluster if their difference 
        is at most 2
        - already_clustered: layers that have an integer value and already correspond to a valid clustering
    """
    clustered_slices = []
    # build one clustering per layer
    for layer in arr.layers:
        x = arr.slice([layer])
        if layer in already_clustered:
            # already a valid clustering, do nothing
            clustered_slices.append(x)
        else:
            # not previously clustered, build a new one using the given tolerated error
            assert layer in err_by_layer, 'Layer has no error and is not already a cluster'
            clustered = regular_partition_clustering(x, err_by_layer[layer])
            clustered_slices.append(clustered)
    # maximal value in all layer-clusterings
    max_val = max([np.amax(x.data.view('int32')) for x in clustered_slices])
    # working copy for building the full clustering
    x = np.array(clustered_slices[0].data.view('int32'))
    # add the contribution of each layer to the working copy
    for i in range(1, len(clustered_slices)):
        x += clustered_slices[i].data.view('int32') * (max_val**i)
    # x is now a full clustering taking into accounts all layers. Make it a geodata and return
    full_cluster = arr.clone(data_array=x, dtype=[(cluster_layer_name, 'int32')])
    return simplify_cluster(full_cluster)


def regular_partition_clustering(arr, max_error):
    """Clusters the given array into slices akin to [0, max_error[, [max_error, 2*max_errror[, ...
    Output is an int32 array of the same shape where each the value of each cell is the identifier of its cluster. 
    """
    if isinstance(arr, GeoData):
        assert len(arr.layers) == 1, "Cannot cluster a GeoData with more than 1 layer"
        x = arr.data.reshape(arr.data.size).astype('float32')
    else:
        x = arr.reshape(arr.size).astype('float32')
    # modify x so that (1) the smallest value is 0 and (2) a variation of max_error becomes a variation of 1
    x = (x - np.min(x)) / max_error
    # clustering is simply done by rounding and casting to int
    clustered = np.round(x).astype('int32')
    if isinstance(arr, GeoData):
        clustered = arr.clone(data_array=clustered.reshape(arr.data.shape), dtype=[('clustered_'+arr.layers[0], 'int32')])
    else:
        clustered = clustered.reshape(arr.shape)
    return simplify_cluster(clustered)


def simplify_cluster(arr):
    """Given a clustering with N clusters, change the clusters ids to [0, N["""
    # extract the array into a 1D array of int32
    if isinstance(arr, GeoData):
        assert len(arr.layers) == 1, "Cannot cluster a GeoData with more than 1 layer"
        x = arr.data.reshape(arr.data.size).astype('int32')
    else:
        x = arr.reshape(arr.size).astype('int32')

    simplified = np.copy(x)
    for k, v in enumerate(np.unique(x)):  # k is the index of the unique value v
        simplified[x == v] = k  # replace v by its index in the unique list

    # put back array in original form
    if isinstance(arr, GeoData):
        clustered = arr.clone(data_array=simplified.reshape(arr.data.shape),
                              dtype=[('clustered_' + arr.layers[0], 'int32')])
    else:
        clustered = simplified.reshape(arr.shape)
    return clustered


def _main():
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    area_wind = (10, np.pi)

    world = World()

    elev = world.get_elevation(area)
    clustered = regular_partition_clustering(elev, 30)  # cluster with a 30 meters margin
    elev.plot()
    clustered.plot()

    wind = world.get_wind(area, domain_average=area_wind)
    clustered = cluster_multi_layer(wind, {'wind_angle': 0.2, 'wind_velocity': 2})
    clustered.plot()

    pyplot.show()  # blocking until all plot windows are closed


if __name__ == '__main__':
    _main()
