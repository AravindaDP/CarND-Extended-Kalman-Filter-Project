import unittest
from ekf.tools import Tools
from ekf.matrix import Matrix
from numpy.testing import assert_array_almost_equal

class TestTools(unittest.TestCase):
    def setUp(self):
        self._tools = Tools()

    def test_calculate_rmse_returns0_if_estimations_size_is0(self):
        estimations = []
        ground_truth = []
        expected_rmse = Matrix([[]])
        expected_rmse.zero(4, 1)

        rmse = self._tools.calculate_rmse(estimations, ground_truth)

        self.assertEqual(rmse.value, expected_rmse.value, 
                         "RMSE should be 0 when estimation size is 0.")

    def test_calculate_rmse_returns0_if_estimations_size_differ(self):
        estimations = [Matrix([[1], [1], [0.2], [0.1]])]
        ground_truth = [Matrix([[1.1], [1.1], [0.3], [0.2]]),
                        Matrix([[2.1], [2.1], [0.4], [0.3]])]
        expected_rmse = Matrix([[]])
        expected_rmse.zero(4, 1)

        rmse = self._tools.calculate_rmse(estimations, ground_truth)

        self.assertEqual(rmse.value, expected_rmse.value, 
                         "RMSE should be 0 when estimation size differs.")

    def test_calculate_jacobian_returns0_if_px_and_py_is0(self):
        state = Matrix([[]])
        state.zero(4, 1)
        expected_Hj = Matrix([[]])
        expected_Hj.zero(3, 4)

        Hj = self._tools.calculate_jacobian(state)

        self.assertEqual(Hj.value, expected_Hj.value, 
                         "Jacobian should be 0 when px and py is 0.")

    def test_calculate_rmse_returns_correct_rmse_for_test_estimations(self):
        for estimations, ground_truth, expected_rmse in [([Matrix([[1], [1], [0.2], [0.1]]),
                                                           Matrix([[2], [2], [0.3], [0.2]]),
                                                           Matrix([[3], [3], [0.4], [0.3]])],
                                                          [Matrix([[1.1], [1.1], [0.3], [0.2]]),
					                                       Matrix([[2.1], [2.1], [0.4], [0.3]]),
									                       Matrix([[3.1], [3.1], [0.5], [0.4]])],
                                                           Matrix([[0.1], [0.1], [0.1], [0.1]]))]:
            with self.subTest():
                rmse = self._tools.calculate_rmse(estimations, ground_truth)

                assert_array_almost_equal(rmse.value, expected_rmse.value, 
                                          err_msg="RMSE should be correctly calculated.")

    def test_calculate_jacobian_returns_correct_Hj_for_test_state(self):
        for state, expected_Hj in [(Matrix([[1], [2], [0.2], [0.4]]),
                                    Matrix([[0.447214, 0.894427, 0, 0],
                                            [-0.4, 0.2, 0, 0],
                                            [0, 0, 0.447214, 0.894427]]))]:
            with self.subTest():
                Hj = self._tools.calculate_jacobian(state)

                assert_array_almost_equal(Hj.value, expected_Hj.value, 
                                          err_msg="RMSE should be correctly calculated.")