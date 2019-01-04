import unittest
from unittest.mock import MagicMock, Mock, call
from ekf.fusion_ekf import FusionEKF
from ekf.tools import Tools
from ekf.kalman_filter import KalmanFilter
from ekf.matrix import Matrix
from ekf.measurement_package import MeasurementPackage
from math import sin, cos
from numpy.testing import assert_array_almost_equal, assert_array_less
import sys, os, csv

class TestFusionEKF(unittest.TestCase):
    def setUp(self):
        self._tools = Tools()
        self._ekf = KalmanFilter()
        sys.stdout = open(os.devnull, 'w')

    def tearDown(self):
        sys.stdout.close()
        sys.stdout = sys.__stdout__
    
    # 1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
    def test_constructor_calls_init_on_ekf(self):
        self._ekf.init = MagicMock()

        fusionEKF = FusionEKF(self._ekf, self._tools)

        self._ekf.init.assert_called_with(Matrix([[0], [0], [0], [0]]),
                                          Matrix([[1, 0, 0, 0],
                                                  [0, 1, 0, 0],
                                                  [0, 0, 1000, 0],
                                                  [0, 0, 0, 1000]]),
                                          Matrix([[1, 0, 1, 0],
                                                  [0, 1, 0, 1],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]]),
                                          Matrix([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]]),
                                          Matrix([[0.0225, 0],
                                                  [0, 0.0225]]),
                                          Matrix([[0, 0, 0, 0],
                                                  [0, 0, 0, 0],
                                                  [0, 0, 0, 0],
                                                  [0, 0, 0, 0]]))

    # 2. initialize the Kalman filter position vector with the first sensor measurements
    def test_process_measurement_sets_x_of_ekf_if_first_measurement_is_LASER(self):
        meas_package = MeasurementPackage(timestamp= 0, sensor_type = MeasurementPackage.SensorType.LASER,
                                          raw_measurements = Matrix([[1], [1]]))

        fusionEKF = FusionEKF(self._ekf, self._tools)
        fusionEKF.process_measurement(meas_package)

        initial_x = Matrix([[1], [1], [0], [0]])

        self.assertEqual(self._ekf._x.value, initial_x.value)

    def test_process_measurement_sets_x_of_ekf_if_first_measurement_is_RADAR(self):
        ro = 1
        theta = 2

        meas_package = MeasurementPackage(timestamp= 0, sensor_type = MeasurementPackage.SensorType.RADAR,
                                          raw_measurements = Matrix([[ro], [theta], [0.5]]))

        fusionEKF = FusionEKF(self._ekf, self._tools)
        fusionEKF.process_measurement(meas_package)

        initial_x = Matrix([[ro*cos(theta)], [ro*sin(theta)], [0], [0]])

        self.assertEqual(self._ekf._x.value, initial_x.value)

    # 3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
    def test_process_measurement_sets_F_and_Q_of_ekf_for_subsequent_measurements(self):
        fusionEKF = FusionEKF(self._ekf, self._tools)

        meas_package = MeasurementPackage(timestamp= 1477010443000000, sensor_type = MeasurementPackage.SensorType.LASER,
                                          raw_measurements =  Matrix([[0.463227], [0.607415]]))
        fusionEKF.process_measurement(meas_package)

        meas_package = MeasurementPackage(timestamp= 1477010443100000, sensor_type = MeasurementPackage.SensorType.LASER,
                                          raw_measurements =  Matrix([[0.968521], [0.40545]]))
        fusionEKF.process_measurement(meas_package)

        self.assertAlmostEqual(self._ekf._F.value[0][2], 0.1)
        self.assertAlmostEqual(self._ekf._F.value[1][3], 0.1)

        dt_2 = 0.01
        dt_3 = 0.001
        dt_4 = 0.0001

        #acceleration noise components
        noise_ax = 9
        noise_ay = 9

        #set the process covariance matrix Q
        #Lesson 5 Section 9
        expected_Q = Matrix([[dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0],
                             [0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay],
			                 [dt_3/2*noise_ax, 0, dt_2*noise_ax, 0],
			                 [0, dt_3/2*noise_ay, 0, dt_2*noise_ay]])

        assert_array_almost_equal(self._ekf._Q.value, expected_Q.value)

    # 4. call the update step for either the lidar or radar sensor measurement. Because the update step
    #    for lidar and radar are slightly different, there are different functions for updating lidar and radar.
    def test_process_measurement_calls_predict_then_update_on_ekf_for_subsequent_LASER_measurements(self):
        self._ekf.predict = MagicMock()
        self._ekf.update = MagicMock()

        ekf_sequence = Mock()
        ekf_sequence.attach_mock(self._ekf.predict, 'predict')
        ekf_sequence.attach_mock(self._ekf.update, 'update')

        fusionEKF = FusionEKF(self._ekf, self._tools)

        meas_package = MeasurementPackage(timestamp= 1477010443000000, sensor_type = MeasurementPackage.SensorType.LASER,
                                          raw_measurements =  Matrix([[0.463227], [0.607415]]))
        fusionEKF.process_measurement(meas_package)

        meas_package = MeasurementPackage(timestamp= 1477010443100000, sensor_type = MeasurementPackage.SensorType.LASER,
                                          raw_measurements =  Matrix([[0.968521], [0.40545]]))
        fusionEKF.process_measurement(meas_package)

        expected_R = Matrix([[0.0225, 0],
                             [0, 0.0225]])
        expected_H = Matrix([[1, 0, 0, 0],
			                 [0, 1, 0, 0]])

        self.assertEqual(self._ekf._R.value, expected_R.value)
        self.assertEqual(self._ekf._H.value, expected_H.value)

        assert ekf_sequence.mock_calls == [call.predict(), call.update(Matrix([[0.968521], [0.40545]]))]

    def test_process_measurement_calls_predict_then_update_ekf_on_ekf_for_subsequent_RADAR_measurements(self):
        self._ekf.predict = MagicMock()
        self._ekf.update_ekf = MagicMock()
        self._tools.calculate_jacobian = MagicMock(return_value=Matrix([[0.8, 0.6, 0, 0],
                                                                        [-0.6, 0.8, 0, 0],
			                                                            [0, 0, 0.8, 0.6]]))

        ekf_sequence = Mock()
        ekf_sequence.attach_mock(self._ekf.predict, 'predict')
        ekf_sequence.attach_mock(self._tools.calculate_jacobian, 'calculate_jacobian')
        ekf_sequence.attach_mock(self._ekf.update_ekf, 'update_ekf')

        fusionEKF = FusionEKF(self._ekf, self._tools)

        meas_package = MeasurementPackage(timestamp= 1477010443050000, sensor_type = MeasurementPackage.SensorType.RADAR,
                                          raw_measurements =  Matrix([[0.898658], [0.617674], [1.7986]]))
        fusionEKF.process_measurement(meas_package)

        meas_package = MeasurementPackage(timestamp= 1477010443150000, sensor_type = MeasurementPackage.SensorType.RADAR,
                                          raw_measurements =  Matrix([[0.910574], [0.610537], [1.46233]]))
        fusionEKF.process_measurement(meas_package)

        expected_R = Matrix([[0.09, 0, 0],
                             [0, 0.0009, 0],
                             [0, 0, 0.09]])
        expected_H = Matrix([[0.8, 0.6, 0, 0],
                             [-0.6, 0.8, 0, 0],
			                 [0, 0, 0.8, 0.6]])

        self.assertEqual(self._ekf._R.value, expected_R.value)
        self.assertEqual(self._ekf._H.value, expected_H.value)

        assert ekf_sequence.mock_calls == [call.predict(), 
                                           call.calculate_jacobian(Matrix([[0.7326109317880749], [0.5204492516937732], [0], [0]])),
                                           call.update_ekf(Matrix([[0.910574], [0.610537], [1.46233]]))]
            
    def test_fusion_ekf_passes_project_rubric_for_dataset1(self):
        in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt"
        
        in_file = open(in_file_name_, newline='')
        data_reader = csv.reader(in_file, delimiter='\t', quotechar='|')

        tools = Tools()
        ekf = KalmanFilter()

        # Create a Fusion EKF instance
        fusionEKF = FusionEKF(ekf, tools)

        # used to compute the RMSE later
        estimations = []
        ground_truth =[]

        # prep the measurement packages (each line represents a measurement at a
        # timestamp)
        for row in data_reader:
            meas_package = MeasurementPackage()

            i = 0
            # reads first element from the current line
            sensor_type = row[i]
            i += 1

            if(sensor_type == "L"):
                # LASER MEASUREMENT

                # read measurements at this timestamp
                meas_package._sensor_type = MeasurementPackage.SensorType.LASER

                px = float(row[i])
                py = float(row[i+1])
                meas_package._raw_measurements = Matrix([[px], [py]])
                timestamp = int(row[i+2])
                meas_package._timestamp = timestamp
                i += 3
            elif (sensor_type == "R"):
                # RADAR MEASUREMENT

                # read measurements at this timestamp
                meas_package._sensor_type = MeasurementPackage.SensorType.RADAR

                ro = float(row[i])
                theta = float(row[i+1])
                ro_dot = float(row[i+2])
                meas_package._raw_measurements = Matrix([[ro], [theta], [ro_dot]])
                timestamp = int(row[i+3])
                meas_package._timestamp = timestamp
                i += 4

            # read ground truth data to compare later
            x_gt = float(row[i])
            y_gt = float(row[i+1])
            vx_gt = float(row[i+2])
            vy_gt = float(row[i+3])

            gt_values =  Matrix([[x_gt], [y_gt], [vx_gt], [vy_gt]])
            ground_truth.append(gt_values)

            fusionEKF.process_measurement(meas_package)
            p_x = fusionEKF._ekf._x.value[0][0]
            p_y = fusionEKF._ekf._x.value[1][0]
            v1  = fusionEKF._ekf._x.value[2][0]
            v2 = fusionEKF._ekf._x.value[3][0]

            estimate = Matrix([[p_x], [p_y], [v1], [v2]])

            estimations.append(estimate)

        # compute the accuracy (RMSE)
        expected_rmse = Matrix([[0.11], [0.11], [0.52], [0.52]])
        rmse = tools.calculate_rmse(estimations, ground_truth)
        assert_array_less(rmse.value, expected_rmse.value)
        
        in_file.close()

    def test_process_measurement_x_and_P_of_ekf_calculated_for_test_measurements(self):
        for measurements, expected_x, expected_P in [([MeasurementPackage(timestamp= 1477010443000000, sensor_type = MeasurementPackage.SensorType.LASER,
                                                                          raw_measurements =  Matrix([[0.463227], [0.607415]])),
                                                       MeasurementPackage(timestamp= 1477010443100000, sensor_type = MeasurementPackage.SensorType.LASER,
                                                                          raw_measurements =  Matrix([[0.968521], [0.40545]])),
                                                       MeasurementPackage(timestamp= 1477010443200000, sensor_type = MeasurementPackage.SensorType.LASER,
                                                                          raw_measurements =  Matrix([[0.947752], [0.636824]])),
                                                       MeasurementPackage(timestamp= 1477010443300000, sensor_type = MeasurementPackage.SensorType.LASER,
                                                                          raw_measurements =  Matrix([[1.42287], [0.264328]]))],
                                                      Matrix([[1.34291], [0.364408], [2.32002], [-0.722813]]),
                                                      Matrix([[0.0185328, 0, 0.109639, 0],
                                                              [0, 0.0185328, 0, 0.109639],
                                                              [0.109639, 0, 1.10798, 0],
                                                              [0, 0.109639, 0, 1.10798]]))]:
            with self.subTest():
                tools = Tools()
                ekf = KalmanFilter()
                
                fusionEKF = FusionEKF(ekf, tools)

                for measurement in measurements:
                    fusionEKF.process_measurement(measurement)

                assert_array_almost_equal(ekf._x.value, expected_x.value, decimal=2)
                assert_array_almost_equal(ekf._P.value, expected_P.value, decimal=1)