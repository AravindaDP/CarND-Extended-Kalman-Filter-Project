#include "gtest/gtest.h"
#include "../src/tools.h"
#include <tuple>
#include <vector>

using Eigen::Map;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowMajor;
using Eigen::VectorXd;

class ToolsTest : public ::testing::Test {
 protected:
  Tools tools_;
};

TEST_F(ToolsTest, CalculateRMSE_Returns0_IfEstimationsSizeIs0) {
  std::vector<VectorXd> estimations;
  std::vector<VectorXd> ground_truth;
  VectorXd expected_rmse = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);

  VectorXd rmse = tools_.CalculateRMSE(estimations, ground_truth);

  ASSERT_TRUE(rmse.isApprox(expected_rmse));
}

TEST_F(ToolsTest, CalculateRMSE_Returns0_IfEstimationsSizeDiffer) {
  std::vector<VectorXd> estimations = {Map<VectorXd>(std::vector<double>({1, 1, 0.2, 0.1}).data(), 4)};
  std::vector<VectorXd> ground_truth = {Map<VectorXd>(std::vector<double>({1.1, 1.1, 0.3, 0.2}).data(), 4),
                                        Map<VectorXd>(std::vector<double>({2.1, 2.1, 0.4, 0.3}).data(), 4)};
  VectorXd expected_rmse = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);

  VectorXd rmse = tools_.CalculateRMSE(estimations, ground_truth);

  ASSERT_TRUE(rmse.isApprox(expected_rmse));
}

TEST_F(ToolsTest, CalculateJacobian_Returns0_IfPxAndPyIs0) {
  VectorXd state = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);
  MatrixXd expected_Hj = MatrixXd::Zero(3,4);

  MatrixXd Hj = tools_.CalculateJacobian(state);

  ASSERT_TRUE(Hj.isApprox(expected_Hj));
}

class CalculateRMSETest: public ToolsTest,
                         public ::testing::WithParamInterface<std::tuple<std::vector<VectorXd>, std::vector<VectorXd>,
						                                                             VectorXd>> {
 public:
  virtual void SetUp() {
	auto test_data = GetParam();
	estimations_ = std::get<0>(test_data);
	ground_truth_ = std::get<1>(test_data);
	expected_rmse_ = std::get<2>(test_data);
  }

 protected:
  std::vector<VectorXd> estimations_;
  std::vector<VectorXd> ground_truth_;
  VectorXd expected_rmse_;
};

TEST_P(CalculateRMSETest, ReturnsCorrectRMSE_ForTestEstimations) {
  VectorXd rmse = tools_.CalculateRMSE(estimations_, ground_truth_);

  ASSERT_TRUE(rmse.isApprox(expected_rmse_));
}

INSTANTIATE_TEST_CASE_P(ToolsTest, CalculateRMSETest, ::testing::Values(
    std::make_tuple(std::vector<VectorXd>({Map<VectorXd>(std::vector<double>({1, 1, 0.2, 0.1}).data(), 4),
                                           Map<VectorXd>(std::vector<double>({2, 2, 0.3, 0.2}).data(), 4),
                                           Map<VectorXd>(std::vector<double>({3, 3, 0.4, 0.3}).data(), 4)}),
                    std::vector<VectorXd>({Map<VectorXd>(std::vector<double>({1.1, 1.1, 0.3, 0.2}).data(), 4),
                                           Map<VectorXd>(std::vector<double>({2.1, 2.1, 0.4, 0.3}).data(), 4),
                                           Map<VectorXd>(std::vector<double>({3.1, 3.1, 0.5, 0.4}).data(), 4)}),
                    Map<VectorXd>(std::vector<double>({0.1, 0.1, 0.1, 0.1}).data(), 4))));

class CalculateJacobianTest: public ToolsTest,
                             public ::testing::WithParamInterface<std::tuple<VectorXd, MatrixXd>> {
 public:
  virtual void SetUp() {
	auto test_data = GetParam();
	state_ = std::get<0>(test_data);
	expected_Hj_ = std::get<1>(test_data);
  }

 protected:
  VectorXd state_;
  MatrixXd expected_Hj_;
};

TEST_P(CalculateJacobianTest, ReturnsCorrectHj_ForTestState) {
  MatrixXd Hj = tools_.CalculateJacobian(state_);

  ASSERT_TRUE(Hj.isApprox(expected_Hj_, 0.0001));
}

INSTANTIATE_TEST_CASE_P(ToolsTest, CalculateJacobianTest, ::testing::Values(
    std::make_tuple(Map<VectorXd>(std::vector<double>({1, 2, 0.2, 0.4}).data(), 4),
                    Map<Matrix<double,3,4,RowMajor>>(std::vector<double>({0.447214, 0.894427, 0, 0,
                                                                          -0.4, 0.2, 0, 0,
                                                                          0, 0, 0.447214, 0.894427}).data()))));