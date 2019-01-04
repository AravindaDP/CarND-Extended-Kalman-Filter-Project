from ekf.matrix import Matrix
from ekf.measurement_package import MeasurementPackage
from math import sin, cos

class FusionEKF:
    def __init__(self, ekf, tools):
        """Constructor"""
        #: obj`KalmanFilter`: Kalman Filter update and prediction math lives in here.
        self._ekf = ekf
        #: obj`Tools`: tool object used to compute Jacobian and RMSE
        self._tools = tools
        #: bool: check whether the tracking toolbox was initialized or not (first measurement)
        self._is_initialized = False
        #: long: previous timestamp
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
        Todo:
            * Finish initializing the FusionEKF.
            * Set the process and measurement noises
        """

        #initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
        #create a 4D state vector, we don't know yet the values of the x state
        x_in = Matrix([[]])
        x_in.zero(4,1)

	    #state covariance matrix P
        P_in = Matrix([[1, 0, 0, 0],
			           [0, 1, 0, 0],
			           [0, 0, 1000, 0],
			           [0, 0, 0, 1000]])


	    #measurement matrix
        self._H_laser = Matrix([[1, 0, 0, 0],
			                    [0, 1, 0, 0]]) # From Lesson 5 Section 10.

	    #the initial transition matrix F_
        F_in = Matrix([[1, 0, 1, 0],
			           [0, 1, 0, 1],
			           [0, 0, 1, 0],
			           [0, 0, 0, 1]])

        Q_in = Matrix([[]])
        Q_in.zero(4, 4)

        self._ekf.init(x_in, P_in, F_in, self._H_laser, self._R_laser, Q_in)

    def process_measurement(self, measurement_pack):
        """Run the whole flow of the Kalman Filter from here."""
        
        """
        Initialization
        """
        if not self._is_initialized:
            """
            Todo:
                * Initialize the state ekf_.x_ with the first measurement.
                * Create the covariance matrix.
            
            You'll need to convert radar from polar to cartesian coordinates.
            """
            # first measurement
            print("EKF: ")
            self._ekf._x = Matrix([[1], [1], [1], [1]])

            #initialize the Kalman filter position vector with the first sensor measurements
            if measurement_pack._sensor_type == MeasurementPackage.SensorType.RADAR:
                """
                Convert radar from polar to cartesian coordinates and initialize state.
                """
                ro = measurement_pack._raw_measurements.value[0][0]
                theta = measurement_pack._raw_measurements.value[1][0]
                self._ekf._x = Matrix([[ro*cos(theta)], [ro*sin(theta)], [0], [0]])
            elif measurement_pack._sensor_type == MeasurementPackage.SensorType.LASER:
                """
                Initialize state.
                """
                #set the state with the initial location and zero velocity
                self._ekf._x = Matrix([measurement_pack._raw_measurements.value[0], measurement_pack._raw_measurements.value[1], [0], [0]])

            self._previous_timestamp = measurement_pack._timestamp

            # done initializing, no need to predict or update
            self._is_initialized  = True
            return

        """
        Prediction
        """

        """
        Todo:
            * Update the state transition matrix F according to the new elapsed time.
        
        Time is measured in seconds.

        Todo:
            * Update the process noise covariance matrix.
        
        Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
        """
        #modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
        #compute the time elapsed between the current and previous measurements
  
        dt = (measurement_pack._timestamp - self._previous_timestamp) / 1000000.0	#dt - expressed in seconds
        self._previous_timestamp = measurement_pack._timestamp

        dt_2 = dt * dt
        dt_3 = dt_2 * dt
        dt_4 = dt_3 * dt

	    #Modify the F matrix so that the time is integrated
        #Lesson 5 Section 8
        self._ekf._F.value[0][2] = dt
        self._ekf._F.value[1][3] = dt

        #acceleration noise components
        noise_ax = 9
        noise_ay = 9

	    #set the process covariance matrix Q
        #Lesson 5 Section 9
        self._ekf._Q = Matrix([[dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0],
                               [0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay],
                               [dt_3/2*noise_ax, 0, dt_2*noise_ax, 0],
                               [0, dt_3/2*noise_ay, 0, dt_2*noise_ay]])

        self._ekf.predict()

        """
        Update
        """

        """
        Todo:
            * Use the sensor type to perform the update step.
            * Update the state and covariance matrices.
        """

        if measurement_pack._sensor_type == MeasurementPackage.SensorType.RADAR:
            # Radar updates

            #set ekf_.H_ by setting to Hj which is the calculated the jacobian
            #set ekf_.R_ by just using R_radar_

            self._Hj = self._tools.calculate_jacobian(self._ekf._x)
            self._ekf._H = self._Hj
            self._ekf._R = self._R_radar

            self._ekf.update_ekf(measurement_pack._raw_measurements)
        else:
            # Laser updates

            #set ekf_.H_ by just using H_laser_
            #set ekf_.R_ by just using R_laser_

            self._ekf._H = self._H_laser
            self._ekf._R = self._R_laser

            self._ekf.update(measurement_pack._raw_measurements)

        # print the output
        print("x_ = ", self._ekf._x)
        print("P_ = ", self._ekf._P)