import numpy as np
from sparse_rrt import _sst_module


def euclidean_distance(is_circular_topology, weights=None):
    '''
    Create computer of weighted euclidean distance between state points.
    Computes the distance based on the topology of the system
    :param is_circular_topology: An array that has flags for each dimensions in the state space whether its circular or not
    :param weights: An array of weights for each coordinate (all ones if None)
    :return: distance computer to pass to a planner
    '''
    if weights is None:
        weights = np.ones_like(is_circular_topology, dtype=np.float64)
    return _sst_module.euclidean_distance(np.asarray(is_circular_topology), np.asarray(weights))


class DistanceGoalSphere(_sst_module.DistanceGoalSphere):
    pass
