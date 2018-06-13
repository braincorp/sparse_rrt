from sparse_rrt.distance_functions import DistanceGoalSphere
from sparse_rrt.planners import RRT
from sparse_rrt.systems import standard_cpp_systems
import numpy as np
import time

from sparse_rrt.systems.acrobot import Acrobot, AcrobotDistance
from sparse_rrt.systems.point import Point


def test_point_rrt():
    '''
    Sanity check RRT test - makes sure that RRT produces exactly(!) the same results during runs.
    Idea is to be deterministic and reproducible and detect when RRT algorithm is changed during refactoring.
    '''
    system = standard_cpp_systems.Point()

    planner = RRT(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        distance=system.distance_computer(),
        start_state=np.array([0., 0.]),
        goal_predicate=DistanceGoalSphere(system.distance_computer(),
                                          goal_state=np.array([9., 9.]),
                                          goal_radius=0.5),
        random_seed=0
    )

    number_of_iterations = 31000

    min_time_steps = 20
    max_time_steps = 200
    integration_step = 0.002

    print("Starting the planner.")

    start_time = time.time()

    expected_results = {
        0: (1, None),
        10000: (7563, 5.4),
        20000: (14936, 4.43),
        30000: (22263, 4.43),
        'final': (22996, 4.43)
    }

    for iteration in range(number_of_iterations):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        if iteration % 10000 == 0:
            solution = planner.get_solution()

            expected_number_of_nodes, expected_solution_cost = expected_results[iteration]

            if solution is None:
                solution_cost = None
                assert(expected_solution_cost is None)
            else:
                _, _, durations, costs = solution
                solution_cost = costs[-1]
                # in this system, costs are equal to cumulative durations
                np.testing.assert_almost_equal(np.sum(durations), costs[-1])
                np.testing.assert_almost_equal(solution_cost, expected_solution_cost)

            print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %s" %
                  (time.time() - start_time, iteration, planner.get_number_of_nodes(), solution_cost))

    path, controls, durations, costs = planner.get_solution()

    print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %f" %
          (time.time() - start_time, number_of_iterations, planner.get_number_of_nodes(), costs[-1]))

    expected_number_of_nodes, expected_solution_cost = expected_results['final']
    assert(planner.get_number_of_nodes() == expected_number_of_nodes)
    np.testing.assert_almost_equal(costs[-1], expected_solution_cost)


def test_create_multiple_times_rrt():
    '''
    There used to be a crash during construction
    '''
    system = standard_cpp_systems.CartPole()
    planners = []
    for i in range(100):
        planner = RRT(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            distance=system.distance_computer(),
            start_state=np.array([-20, 0, 3.14, 0]),
            goal_predicate=DistanceGoalSphere(system.distance_computer(),
                                              goal_state=np.array([20, 0, 3.14, 0]),
                                              goal_radius=1.5),
            random_seed=0
        )
        min_time_steps = 10
        max_time_steps = 50
        integration_step = 0.02

        for iteration in range(100):
            planner.step(system, min_time_steps, max_time_steps, integration_step)
        planners.append(planner)


def test_py_system_rrt():

    system = Point()

    planner = RRT(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        distance=system.distance_computer(),
        start_state=np.array([0.2, 0.1]),
        goal_predicate=DistanceGoalSphere(system.distance_computer(),
                                          goal_state=np.array([5., 5.]),
                                          goal_radius=1.5),
        random_seed=0
    )

    min_time_steps = 10
    max_time_steps = 50
    integration_step = 0.02

    for iteration in range(1000):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        im = planner.visualize_tree(system)


def test_py_system_rrt_custom_distance():
    '''
    Check that distance overriding in python works
    '''

    system = Acrobot()

    planner = RRT(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        # use custom distance computer
        distance=AcrobotDistance(),
        start_state=np.array([0., 0., 0., 0.]),
        goal_predicate=DistanceGoalSphere(AcrobotDistance(),
                                          goal_state=np.array([np.pi, 0., 0., 0.]),
                                          goal_radius=2.),
        random_seed=0
    )

    min_time_steps = 10
    max_time_steps = 50
    integration_step = 0.02

    for iteration in range(100):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        im = planner.visualize_tree(system)


def test_multiple_runs_same_result():
    '''
    Test that runs are deterministic
    '''
    system = standard_cpp_systems.Point()

    def _create_planner():
        return RRT(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            distance=system.distance_computer(),
            start_state=np.array([0., 0.]),
            goal_predicate=DistanceGoalSphere(AcrobotDistance(),
                                              goal_state=np.array([9., 9.]),
                                              goal_radius=0.5),
            random_seed=0
        )

    planner = _create_planner()
    for i in range(1000):
        planner.step(system, 10, 50, 0.02)
    original_number_of_nodes = planner.get_number_of_nodes()

    for i in range(5):
        planner = _create_planner()
        for i in range(1000):
            planner.step(system, 10, 50, 0.02)
        assert original_number_of_nodes == planner.get_number_of_nodes()


if __name__ == '__main__':
    test_create_multiple_times_rrt()
    test_py_system_rrt()
    test_py_system_rrt_custom_distance()
    st = time.time()
    test_point_rrt()
    print("Current test time: %fs (baseline: %fs)" % (time.time() - st, 21.4076721668))
    test_multiple_runs_same_result()
    print('Passed all tests!')
