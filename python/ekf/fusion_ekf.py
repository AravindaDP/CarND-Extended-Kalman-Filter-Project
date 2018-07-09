from ekf.matrix import Matrix
from ekf.measurement_package import MeasurementPackage
from math import sin, cos

class FusionEKF:
    def __init__(self, ekf, tools):
        """Constructor"""
        self._ekf = ekf
        self._tools = tools

        self._is_initialized = False

        self._previous_timestamp = 0

        #initializing matrices
        self._R_laser = Matrix([[]])
        self._R_radar = Matrix([[]])
        self._H_laser_ = Matrix([[]])
        self._Hj = Matrix([[]])
        self._Hj.zero(3, 4)

        #measurement covariance matrix - laser
        self._R_laser = Matrix([[0.0225, 0],
                                [0, 0.0225]])

        #measurement covariance matrix - radar
        self._R_radar = Matrix([[0.09, 0, 0],
                                [0, 0.0009, 0],
                                [0, 0, 0.09]])

        """
        # TODO:
        # Finish initializing the FusionEKF.
        # Set the process and measurement noises
        """

    def process_measurement(self, measurement_pack):
        """Run the whole flow of the Kalman Filter from here."""

        """
        # Initialization
        """
        if not self._is_initialized:
            """
            # TODO:
            # Initialize the state ekf_.x_ with the first measurement.
            # Create the covariance matrix.
            # Remember: you'll need to convert radar from polar to cartesian coordinates.
            """
            # first measurement
            print("EKF: ")
            self._ekf._x = Matrix([[1], [1], [1], [1]])

            #initialize the Kalman filter position vector with the first sensor measurements
            if measurement_pack._sensor_type == MeasurementPackage.SensorType.RADAR:
                """
                Convert radar from polar to cartesian coordinates and initialize state.
                """
            elif measurement_pack._sensor_type == MeasurementPackage.SensorType.LASER:
                """
                Initialize state.
                """

            # done initializing, no need to predict or update
            self._is_initialized  = True
            return

        """
        # Prediction
        """

        """
        # TODO:
        # Update the state transition matrix F according to the new elapsed time.
        # - Time is measured in seconds.
        # Update the process noise covariance matrix.
        # Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
        """

        self._ekf.predict()

        """
        #  Update
        """

        """
        # TODO:
        # Use the sensor type to perform the update step.
        # Update the state and covariance matrices.
        """

        if measurement_pack._sensor_type == MeasurementPackage.SensorType.RADAR:
            # Radar updates
            pass
        else:
            # Laser updates
            pass

        # print the output
        print("x_ = ", self._ekf._x)
        print("P_ = ", self._ekf._P)