import numpy as np
import pytest
from sparse_rrt.distance_functions import euclidean_distance, DistanceGoalSphere


def test_euclidean_distance_flat():
    dist = euclidean_distance([False, False])

    np.testing.assert_almost_equal(dist.distance([0., 0.], [1., 1]), np.sqrt(2))
    np.testing.assert_almost_equal(dist.distance([0., 0.], [1., 0.]), 1)
    np.testing.assert_almost_equal(dist.distance([-1., 0.], [1., 0.]), 2.)
    # 2d distance should fail on different dimension points
    with pytest.raises(RuntimeError):
        dist.distance([0.,], [1.,])

    # shape mismatch
    with pytest.raises(RuntimeError):
        dist.distance([0., 0.], [1., ])


def test_euclidean_distance_circular():
    dist = euclidean_distance([True, False])
    np.testing.assert_almost_equal(dist.distance([0., 0.], [1., 1]), np.sqrt(2))
    np.testing.assert_almost_equal(dist.distance([2 * np.pi, 0.], [0., 0.]), 0.)
    np.testing.assert_almost_equal(dist.distance([0., 0.], [0., 2*np.pi]), 2*np.pi)

    np.testing.assert_almost_equal(dist.distance([0.1, 0.1], [2*np.pi - 0.1, -0.1]), 0.2*np.sqrt(2))


def test_euclidean_distance_weighted():
    dist = euclidean_distance([False, False], weights=[1., 10.])
    np.testing.assert_almost_equal(dist.distance([0., 0.], [1., 0.]), 1)
    np.testing.assert_almost_equal(dist.distance([0., 0.], [0., 1.]), 10.)

    # shape mismatch
    with pytest.raises(RuntimeError):
        dist = euclidean_distance([True, False, False], weights=[1., 0.1])

    dist = euclidean_distance([False, True, False], weights=[1., 0.1, 10.])
    np.testing.assert_almost_equal(dist.distance([-1., 1., 0.], [0., np.pi*2, 0.5]), np.sqrt(0.01 + 1 + 10*10*0.5*0.5))


def test_goal_predicate():
    goal_predicate = DistanceGoalSphere(euclidean_distance([False, False]), [2., -2.], 1.)

    assert not goal_predicate.reached_goal([0., 0.])
    assert not goal_predicate.reached_goal([2., 0.])
    assert goal_predicate.reached_goal([2., -2.])
    assert goal_predicate.reached_goal([2., -1.])
    assert not goal_predicate.reached_goal([1.9, -1.])

    # goal predicate that ignores the second coordinate
    goal_predicate = DistanceGoalSphere(euclidean_distance([False, False], weights=[1., 0.]), [2., -2.], 1.)

    assert not goal_predicate.reached_goal([0., 0.])
    assert goal_predicate.reached_goal([2., 0.])
    assert goal_predicate.reached_goal([2., -2.])
    assert goal_predicate.reached_goal([2., -1.])
    assert goal_predicate.reached_goal([1.9, -1.])


if __name__ == '__main__':
    test_euclidean_distance_flat()
    test_euclidean_distance_circular()
    test_euclidean_distance_weighted()
    test_goal_predicate()
