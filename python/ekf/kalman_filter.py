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
        self._x = self._F * self._x # Lesson 5 Section 8
        Ft = self._F.transpose()
        self._P = self._F * self._P * Ft + self._Q # Lesson 5 Section 9

    def update(self, z):
        """Updates the state by using standard Kalman Filter equations
            
            z: The measurement at k+1
        """
        """
        # TODO:
        # update the state by using Kalman Filter equations
        """
        z_pred = self._H * self._x
        y = z - z_pred
        Ht = self._H.transpose()
        S = self._H * self._P * Ht + self._R
        Si = S.inverse()
        PHt = self._P * Ht
        K = PHt * Si

	    #new estimate
        self._x = self._x + (K * y)
        x_size = self._x.dimx
        I = Matrix([[]])
        I.identity(x_size)
        self._P = (I - K * self._H) * self._P

    def update_ekf(self, z):
        """Updates the state by using Extended Kalman Filter equations
   
            z: The measurement at k+1
        """
        """
        # TODO:
        # update the state by using Extended Kalman Filter equations
        """
        #Lesson 5 Section 14
        px = self._x.value[0][0]
        py = self._x.value[1][0]
        vx = self._x.value[2][0]
        vy = self._x.value[3][0]

        rho = sqrt(px*px+py*py)
        theta = atan2(py,px)
        ro_dot = (px*vx+py*vy)/rho
        z_pred = Matrix([[rho], [theta], [ro_dot]])

        y = z - z_pred
        if(y.value[1][0] > pi):
            y.value[1][0] -= 2*pi
        elif(y.value[1][0]<(-pi)):
            y.value[1][0] += 2*pi

        #Lesson 5 Section 7
        Ht = self._H.transpose()
        S = self._H * self._P * Ht + self._R
        Si = S.inverse()
        PHt = self._P * Ht
        K = PHt * Si

        #new estimate
        self._x = self._x + (K * y)
        x_size = self._x.dimx
        I = Matrix([[]])
        I.identity(x_size)
        self._P = (I - K * self._H) * self._P
