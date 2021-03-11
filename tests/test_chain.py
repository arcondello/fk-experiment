import math
import unittest

import numpy as np

from chain import JointChain


class TestIntegration(unittest.TestCase):
    def test_3joint(self):
        jc = JointChain()

        self.assertEqual(jc.num_joints(), 1)

        j0 = jc.base()
        j1 = jc.add_joint([1, 0, 0], [0, -.707, 0, .707])
        j2 = jc.add_joint([1, 1, 0], [0, 0, .707, .707])

        self.assertEqual(jc.num_joints(), 3)

        np.testing.assert_array_almost_equal(j1.position, [1, 0, 0])
        np.testing.assert_array_almost_equal(j1.orientation, [0, -.707, 0, .707])
        np.testing.assert_array_almost_equal(j2.position, [1, 1, 0])
        np.testing.assert_array_almost_equal(j2.orientation, [0, 0, .707, .707])

        j0.set_angle(math.pi / 2)

        self.assertAlmostEqual(j0.get_angle(), math.pi / 2)

        np.testing.assert_array_almost_equal(j1.position, [1, 0, 0])
        np.testing.assert_array_almost_equal(j1.orientation, [.5, -.5, -.5, .5])
        np.testing.assert_array_almost_equal(j2.position, [1, 0, 1])
        np.testing.assert_array_almost_equal(j2.orientation, [.5, -.5, .5, .5])

        # after this it does not work very well
