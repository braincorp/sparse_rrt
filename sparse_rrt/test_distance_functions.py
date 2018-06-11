import numpy as np
import pytest
from sparse_rrt.distance_functions import euclidean_distance


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


if __name__ == '__main__':
    test_euclidean_distance_flat()
    test_euclidean_distance_circular()
