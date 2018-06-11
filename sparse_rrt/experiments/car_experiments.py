import numpy as np
import sparse_rrt
from sparse_rrt.distance_functions import euclidean_distance, DistanceGoalSphere
from sparse_rrt.experiments.experiment_utils import run_config

# config for experiments with Car
from sparse_rrt.systems import Car
from sparse_rrt.utilities import normalize_angle


class CarGoal(sparse_rrt._sst_module.IGoalPredicate):
    '''
    A goal predicate that has independent thresholds for spatial and angular distances
    '''
    def __init__(self, goal_state, spatial_threshold, angular_threshold):
        '''
        :param goal_state: goal state in the state space (x, y, angle)
        :param spatial_threshold: spatial distance must be lower than this
        :param angular_threshold: angular distance must be lower that this
        '''
        sparse_rrt._sst_module.IGoalPredicate.__init__(self)
        self._goal_state = goal_state
        self._spatial_threshold = spatial_threshold
        self._angular_threshold = angular_threshold

    def reached_goal(self, point):
        spatial_dist = np.linalg.norm(self._goal_state[:2]-point[:2])
        angular_dist = np.abs(normalize_angle(self._goal_state[2] - point[2]))
        return spatial_dist <= self._spatial_threshold and angular_dist <= self._angular_threshold


base_car_config = dict(
    start_state=[0., 0., 0.],
    random_seed=0,
    sst_delta_near=0.6,
    sst_delta_drain=0.2,
    integration_step=0.002,
    min_time_steps=20,
    max_time_steps=200,
    debug_period=1000,
    distance_computer=euclidean_distance(
        Car().is_circular_topology(),
        weights=[1., 1., 1.]
    ),
    goal_predicate=CarGoal(
        goal_state=[9., 0., -np.pi/2],
        spatial_threshold=0.5,
        angular_threshold=0.1
    ),
    number_of_iterations=300000,
    display_type='tree'
)

# different configs for cpp and py implementations of the system
cpp_car_config = dict(system='car', **base_car_config)
py_car_config = dict(system='py_car', **base_car_config)

# different configs with different planners
rrt_car_config = dict(planner='rrt', **cpp_car_config)
sst_car_config = dict(planner='sst', **cpp_car_config)
rrt_py_car_config = dict(planner='rrt', **py_car_config)
sst_py_car_config = dict(planner='sst', **py_car_config)


if __name__ == '__main__':
    run_config(sst_car_config)
