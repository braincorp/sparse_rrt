import numpy as np
from sparse_rrt import _sst_module


def euclidean_distance(is_circular_topology):
    '''

    :param is_circular_topology:
    :return:
    '''
    return _sst_module.euclidean_distance(np.asarray(is_circular_topology))
