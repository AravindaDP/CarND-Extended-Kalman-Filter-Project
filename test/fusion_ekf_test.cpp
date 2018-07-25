#include "../src/FusionEKF.h"
#include "mock_kalman_filter.h"
#include "mock_tools.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Invoke;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Eq;
using ::testing::DoubleNear;
using ::testing::Return;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::RowMajor;
using std::sin;
using std::cos;
using std::vector;
using std::ifstream;

MATCHER_P(IsApprox, n, "") { return arg.isApprox(n, 0.0001); }
MATCHER_P(IsLt, n, "") { return ((arg-n).array() < 0).any(); }

class FusionEKFTest : public ::testing::Test {
 protected:
  MockKalmanFilter ekf_;
  MockTools tools_;
  
  void SetUp() override {
    EXPECT_CALL(ekf_, Init(_, _, _, _, _, _))
        .Times(AtMost(1))
        .WillOnce(Invoke(&ekf_, &MockKalmanFilter::KalmanFilterInit)); // Supress warning on un interested places
    EXPECT_CALL(ekf_, Predict()).Times(AtMost(1));
    EXPECT_CALL(ekf_, Update(_)).Times(AtMost(1));
    EXPECT_CALL(ekf_, UpdateEKF(_)).Times(AtMost(1));

    std::cout.setstate(std::ios_base::failbit);
  }

  void TearDown() override {
    std::cout.clear();  
  }
};

// 1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
TEST_F(FusionEKFTest, Constructor_CallsInitOnEKF) {
  EXPECT_CALL(ekf_, Init(Eq(VectorXd::Zero(4)),
                         Eq((MatrixXd)Map<Matrix<double,4,4,RowMajor>>(vector<double>({1, 0, 0, 0,
                                                                                       0, 1, 0, 0,
                                                                                       0, 0, 1000, 0,
                                                                                       0, 0, 0, 1000}).data())),
                         Eq((MatrixXd)Map<Matrix<double,4,4,RowMajor>>(vector<double>({1, 0, 1, 0,
                                                                                       0, 1, 0, 1,
                                                                                       0, 0, 1, 0,
                                                                                       0, 0, 0, 1}).data())),
                         Eq((MatrixXd)Map<Matrix<double,2,4,RowMajor>>(vector<double>({1, 0, 0, 0,
                                                                                       0, 1, 0, 0}).data())),
                         Eq((MatrixXd)Map<Matrix<double,2,2,RowMajor>>(vector<double>({0.0225, 0,
                                                                                       0, 0.0225}).data())),
                         Eq(MatrixXd::Zero(4,4))));

  FusionEKF fusionEKF(ekf_, tools_);
}

// 2. initialize the Kalman filter position vector with the first sensor measurements
TEST_F(FusionEKFTest, ProcessMeasurement_SetsXOfEKF_IfFirstMeasurementIsLASER) {
	MeasurementPackage meas_package = {0, MeasurementPackage::LASER,
                                       Map<VectorXd>(std::vector<double>({ 1, 1 }).data(), 2) };

  FusionEKF fusionEKF(ekf_, tools_);
  fusionEKF.ProcessMeasurement(meas_package);

  VectorXd initial_x_ = VectorXd(4);
  initial_x_ << 1, 1, 0, 0;

  ASSERT_TRUE(ekf_.x_.isApprox(initial_x_));
}

TEST_F(FusionEKFTest, ProcessMeasurement_SetsXOfEKF_IfFirstMeasurementIsRADAR) {
  float ro = 1;
  float theta = 2;

  MeasurementPackage meas_package = {0, MeasurementPackage::RADAR,
                                     Map<VectorXd>(vector<double>({ro, theta, 0.5}).data(), 3)};

  FusionEKF fusionEKF(ekf_, tools_);
  fusionEKF.ProcessMeasurement(meas_package);

  VectorXd initial_x_ = VectorXd(4);
  initial_x_ << ro*cos(theta), ro*sin(theta), 0, 0;

  ASSERT_TRUE(ekf_.x_.isApprox(initial_x_));
}

// 3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
TEST_F(FusionEKFTest, ProcessMeasurement_SetsFAndQOfEKF_ForSubsequentMeasurements) {
  MeasurementPackage meas_package;

  FusionEKF fusionEKF(ekf_, tools_);

  meas_package = {1477010443000000, MeasurementPackage::LASER,
                  Map<VectorXd>(vector<double>({0.463227, 0.607415}).data(), 2)};
  fusionEKF.ProcessMeasurement(meas_package);

  meas_package = {1477010443100000, MeasurementPackage::LASER,
                  Map<VectorXd>(vector<double>({0.968521, 0.40545}).data(), 2)};
  fusionEKF.ProcessMeasurement(meas_package);

  ASSERT_THAT(ekf_.F_(0, 2), DoubleNear(0.1, 0.0001));
  ASSERT_THAT(ekf_.F_(1, 3), DoubleNear(0.1, 0.0001));

  float dt_2 = 0.01;
  float dt_3 = 0.001;
  float dt_4 = 0.0001;

  //acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  //set the process covariance matrix Q
  //Lesson 5 Section 9
  MatrixXd expected_Q = MatrixXd(4, 4);
  expected_Q <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                 0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                 dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                 0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ASSERT_TRUE(ekf_.Q_.isApprox(expected_Q, 0.0001));
}

// 4. call the update step for either the lidar or radar sensor measurement. Because the update step
//    for lidar and radar are slightly different, there are different functions for updating lidar and radar.
TEST_F(FusionEKFTest, ProcessMeasurement_CallsPredictThenUpdateOnEKF_ForSubsequentLASERMeasurements) {
  {
    InSequence ekf;

    EXPECT_CALL(ekf_, Predict());
    EXPECT_CALL(ekf_, Update((VectorXd)Map<VectorXd>(vector<double>({0.968521, 0.40545}).data(), 2)));
  }

  MeasurementPackage meas_package;

  FusionEKF fusionEKF(ekf_, tools_);

  meas_package = {1477010443000000, MeasurementPackage::LASER,
                  Map<VectorXd>(vector<double>({0.463227, 0.607415}).data(), 2)};
  fusionEKF.ProcessMeasurement(meas_package);

  meas_package = {1477010443100000, MeasurementPackage::LASER,
                  Map<VectorXd>(vector<double>({0.968521, 0.40545}).data(), 2)};
  fusionEKF.ProcessMeasurement(meas_package);

  MatrixXd expected_R = MatrixXd(2, 2);
  MatrixXd expected_H = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  expected_R << 0.0225, 0,
                0, 0.0225;

  expected_H << 1, 0, 0, 0,
                0, 1, 0, 0;

  ASSERT_TRUE(ekf_.R_.isApprox(expected_R, 0.0001));
  ASSERT_TRUE(ekf_.H_.isApprox(expected_H, 0.0001));
}

TEST_F(FusionEKFTest, ProcessMeasurement_CallsPredictThenUpdateEKFOnEKF_ForSubsequentRADARMeasurements) {
  {
    InSequence ekf;

    EXPECT_CALL(ekf_, Predict());
    EXPECT_CALL(tools_,
                CalculateJacobian(IsApprox((VectorXd)Map<VectorXd>(vector<double>({0.732611, 0.520449, 0, 0}).data(), 4))))
        .WillOnce(Return(Map<Matrix<double,3,4,RowMajor>>(vector<double>({0.8, 0.6, 0, 0,
                                                                          -0.6, 0.8, 0, 0,
                                                                          0, 0, 0.8, 0.6}).data())));
    EXPECT_CALL(ekf_, UpdateEKF((VectorXd)Map<VectorXd>(vector<double>({0.910574, 0.610537, 1.46233}).data(), 3)));
  }

  MeasurementPackage meas_package;

  FusionEKF fusionEKF(ekf_, tools_);

  meas_package = {1477010443050000, MeasurementPackage::RADAR,
                  Map<VectorXd>(vector<double>({0.898658, 0.617674, 1.7986}).data(), 3)};
  fusionEKF.ProcessMeasurement(meas_package);

  meas_package = {1477010443150000, MeasurementPackage::RADAR,
                  Map<VectorXd>(vector<double>({0.910574, 0.610537, 1.46233}).data(), 3)};
  fusionEKF.ProcessMeasurement(meas_package);

  MatrixXd expected_R = MatrixXd(3, 3);
  MatrixXd expected_H = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  expected_R << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

  expected_H << 0.8, 0.6, 0, 0,
                -0.6, 0.8, 0, 0,
                0, 0, 0.8, 0.6;

  ASSERT_TRUE(ekf_.R_.isApprox(expected_R, 0.0001));
  ASSERT_TRUE(ekf_.H_.isApprox(expected_H, 0.0001));
}

TEST_F(FusionEKFTest, FusionEKF_PassesProjectRubric_ForDataSet1) {
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  ASSERT_TRUE(in_file_.is_open());

  string line;

  Tools tools;
  KalmanFilter ekf;

  // Create a Fusion EKF instance
  FusionEKF fusionEKF(ekf, tools);

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    VectorXd gt_values(4);
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
    }
    else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_values << x_gt, y_gt, vx_gt, vy_gt;
    ground_truth.push_back(gt_values);

    fusionEKF.ProcessMeasurement(meas_package);
    estimations.push_back(fusionEKF.ekf_.x_);
  }

  // compute the accuracy (RMSE)
  VectorXd expected_rmse(4);
  expected_rmse << 0.11, 0.11, 0.52, 0.52;
  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
  ASSERT_THAT(tools.CalculateRMSE(estimations, ground_truth), IsLt(expected_rmse));

  // close files
  if (in_file_.is_open()) {
    in_file_.close();
  }
}

class ProcessMeasurementTest : public FusionEKFTest,
                               public ::testing::WithParamInterface<std::tuple<vector<MeasurementPackage>, VectorXd, 
                                                                               MatrixXd>> {
 public:
  virtual void SetUp() {
    auto test_data = GetParam();
    measurements_ = std::get<0>(test_data);
    expected_x_ = std::get<1>(test_data);
    expected_P_ = std::get<2>(test_data);

    std::cout.setstate(std::ios_base::failbit);
  }

 protected:
  vector<MeasurementPackage> measurements_;
  // expected state
  VectorXd expected_x_;
  MatrixXd expected_P_;

  void TearDown() override {
    std::cout.clear();
  }
};

TEST_P(ProcessMeasurementTest, XAndPOfEKFCalculated_ForTestMeasurements) {
  Tools tools;
  KalmanFilter ekf;

  FusionEKF fusionEKF(ekf, tools);

  for (unsigned int i = 0; i < measurements_.size(); ++i) {
    fusionEKF.ProcessMeasurement(measurements_[i]);
  }

  ASSERT_TRUE(ekf.x_.isApprox(expected_x_, 0.1));
  ASSERT_TRUE(ekf.P_.isApprox(expected_P_, 0.1));
}

INSTANTIATE_TEST_CASE_P(FusionEKFTest, ProcessMeasurementTest, ::testing::Values(
    std::make_tuple(vector<MeasurementPackage>({{1477010443000000, MeasurementPackage::LASER,
                                                 Map<VectorXd>(vector<double>({0.463227, 0.607415}).data(), 2)},
                                                {1477010443100000, MeasurementPackage::LASER,
                                                 Map<VectorXd>(vector<double>({0.968521, 0.40545}).data(), 2)},
                                                {1477010443200000, MeasurementPackage::LASER,
                                                 Map<VectorXd>(vector<double>({0.947752, 0.636824}).data(), 2)},
                                                {1477010443300000, MeasurementPackage::LASER,
                                                 Map<VectorXd>(vector<double>({ 1.42287, 0.264328 }).data(), 2)}}),
                    Map<VectorXd>(std::vector<double>({1.34291, 0.364408, 2.32002, -0.722813}).data(), 4),
                    Map<Matrix<double, 4, 4, RowMajor>>(std::vector<double>({0.0185328, 0, 0.109639, 0,
                                                                             0, 0.0185328, 0, 0.109639,
                                                                             0.109639, 0, 1.10798, 0,
                                                                             0, 0.109639, 0, 1.10798 }).data()))));