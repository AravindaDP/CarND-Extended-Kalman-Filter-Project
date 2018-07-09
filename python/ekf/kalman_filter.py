from ekf.matrix import Matrix
from math import sqrt, atan2, pi

# Please note that the Matrix class does not initialize
# Matrix objects with zeros upon creation.

class KalmanFilter :
    def __init__(self):
        """Constructor"""
        # state vector
        self._x = Matrix([[]])

        # state covariance matrix
        self._P = Matrix([[]])

        # state transition matrix
        self._F = Matrix([[]])

        # process covariance matrix
        self._Q = Matrix([[]])

        # measurement matrix
        self._H = Matrix([[]])

        # measurement covariance matrix
        self._R = Matrix([[]])

    def init(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        """Initializes Kalman filter

            x_in: Initial state
            P_in: Initial state covariance
            F_in: Transition matrix
            H_in: Measurement matrix
            R_in: Measurement covariance matrix
            Q_in: Process covariance matrix
        """
        self._x = x_in
        self._P = P_in
        self._F = F_in
        self._H = H_in
        self._R = R_in
        self._Q = Q_in

    def predict(self):
        """Predicts the state and the state covariance using the process model

            delta_T: Time between k and k+1 in s
        """
        """
        # TODO:
        # predict the state
        """

    def update(self, z):
        """Updates the state by using standard Kalman Filter equations

            z: The measurement at k+1
        """
        """
        # TODO:
        # update the state by using Kalman Filter equations
        """

    def update_ekf(self, z):
        """Updates the state by using Extended Kalman Filter equations

            z: The measurement at k+1
        """
        """
        # TODO:
        # update the state by using Extended Kalman Filter equations
        """
