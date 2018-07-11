from ekf.matrix import Matrix
from math import sqrt

class Tools:
    def calculate_rmse(self, estimations, ground_truth):
        """
        # TODO:
        # Calculate the RMSE here.
        """
        rmse = Matrix([[]])
        rmse.zero(4, 1)

        # check the validity of the following inputs:
        # * the estimation vector size should not be zero
        # * the estimation vector size should equal ground truth vector size
        if len(estimations) != len(ground_truth) or not estimations:
            print("Invalid estimation or ground_truth data")
            return rmse

        #accumulate squared residuals
        for i in range(len(estimations)):
            residual = estimations[i] - ground_truth[i]

            #coefficient-wise multiplication
            residual = residual.cwise_product(residual)
            rmse += residual

        #calculate the mean
        rmse.value = [[i[0]/len(estimations)] for i in rmse.value]

        #calculate the squared root
        rmse.value = [[sqrt(i[0])] for i in rmse.value]

        #return the result
        return rmse

    def calculate_jacobian(self, x_state):
        """
        # TODO:
        # Calculate a Jacobian here.
        """
        Hj = Matrix([[]])
        Hj.zero(3, 4)
        #recover state parameters
        px = x_state.value[0][0]
        py = x_state.value[1][0]
        vx = x_state.value[2][0]
        vy = x_state.value[3][0]

        #pre-compute a set of terms to avoid repeated calculation
        c1 = px*px+py*py
        c2 = sqrt(c1)
        c3 = (c1*c2)

        #check division by zero
        if(abs(c1) < 0.0001):
            print("CalculateJacobian () - Error - Division by Zero")
            return Hj

        #compute the Jacobian matrix
        Hj = Matrix([[(px/c2), (py/c2), 0, 0],
                     [-(py/c1), (px/c1), 0, 0],
		             [py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2]])

        return Hj
