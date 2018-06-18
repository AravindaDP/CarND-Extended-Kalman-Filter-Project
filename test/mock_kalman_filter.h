#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/kalman_filter.h"

class MockKalmanFilter : public KalmanFilter {
 public:
  MOCK_METHOD6(Init, void(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                          MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in));
  MOCK_METHOD0(Predict, void());
  MOCK_METHOD1(Update, void(const Eigen::VectorXd &z));
  MOCK_METHOD1(UpdateEKF, void(const Eigen::VectorXd &z));

  // Use this to call Init() defined in KalmanFilter.
  void KalmanFilterInit(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) { 
    return KalmanFilter::Init(x_in, P_in, F_in, H_in, R_in, Q_in);
  }
};