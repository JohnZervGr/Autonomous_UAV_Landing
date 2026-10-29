import unittest

from drone_control.kallman import KF

import numpy as np


class TestKFInitialization(unittest.TestCase):
    """Tests for Kalman Filter initialization."""

    def setUp(self):
        self.kf = KF(sigma_a=1.0, s=1.0)

    def test_initial_state(self):
        np.testing.assert_array_equal(
            self.kf.x,
            np.zeros(6)
        )

    def test_initial_covariance(self):
        np.testing.assert_array_equal(
            self.kf.P,
            np.eye(6)
        )

    def test_transition_matrix(self):
        expected_F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        np.testing.assert_array_equal(
            self.kf.F,
            expected_F
        )

    def test_control_matrix(self):
        expected_B = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])

        np.testing.assert_array_equal(
            self.kf.B,
            expected_B
        )
