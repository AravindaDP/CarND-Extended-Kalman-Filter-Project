#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/kalman_filter.h"

class MockKalmanFilter : public KalmanFilter {
 public:
  MOCK_METHOD6(Init, void(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                          Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in));
  MOCK_METHOD0(Predict, void());
  MOCK_METHOD1(Update, void(const Eigen::VectorXd &z));
  MOCK_METHOD1(UpdateEKF, void(const Eigen::VectorXd &z));

  // Use this to call Init() defined in KalmanFilter.
  void KalmanFilterInit(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
    return KalmanFilter::Init(x_in, P_in, F_in, H_in, R_in, Q_in);
  }
};