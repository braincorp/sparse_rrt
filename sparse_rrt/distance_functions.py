import numpy as np
from sparse_rrt import _sst_module


def euclidean_distance(is_circular_topology, weights=None):
    '''

    :param is_circular_topology:
    :return:
    '''
    if weights is None:
        weights = np.ones_like(is_circular_topology, dtype=np.float64)
    return _sst_module.euclidean_distance(np.asarray(is_circular_topology), np.asarray(weights))
