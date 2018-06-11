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
    def __init__(self, distance_computer, goal_state, goal_radius):
        '''
        Create the most common goal predicate -
        goal is reached if distance between the state and the goal point is withing the radius
        :param distance_computer: an object that computes distance for the system
        :param goal_state: goal state point
        :param goal_radius: radius around goal state point
        '''
        _sst_module.DistanceGoalSphere.__init__(self, distance_computer, goal_state, goal_radius)
