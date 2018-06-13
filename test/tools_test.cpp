#include "gtest/gtest.h"
#include "../src/tools.h"
#include <tuple>
#include <vector>

using Eigen::Map;
using Eigen::Matrix;
using Eigen::RowMajor;

class ToolsTest : public ::testing::Test {
 protected:
  Tools tools_;
};

TEST_F(ToolsTest, CalculateRMSE_Return0_IfEstimationSize0) {
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  VectorXd expected_rmse = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);

  VectorXd rmse = tools_.CalculateRMSE(estimations, ground_truth);

  ASSERT_TRUE(rmse.isApprox(expected_rmse));
}

TEST_F(ToolsTest, CalculateRMSE_Return0_IfEstimationSizeDiffer) {
  vector<VectorXd> estimations = {Map<VectorXd>(vector<double>({1, 1, 0.2, 0.1}).data(), 4)};
  vector<VectorXd> ground_truth = {Map<VectorXd>(vector<double>({1.1, 1.1, 0.3, 0.2}).data(), 4),
                                   Map<VectorXd>(vector<double>({2.1, 2.1, 0.4, 0.3}).data(), 4)};
  VectorXd expected_rmse = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);

  VectorXd rmse = tools_.CalculateRMSE(estimations, ground_truth);

  ASSERT_TRUE(rmse.isApprox(expected_rmse));
}

TEST_F(ToolsTest, CalculateJacobian_Return0_IfPxAndPyIs0) {
  VectorXd state = Map<VectorXd>(std::vector<double>({0, 0, 0, 0}).data(), 4);
  MatrixXd expected_Hj = MatrixXd::Zero(3,4);

  MatrixXd Hj = tools_.CalculateJacobian(state);

  ASSERT_TRUE(Hj.isApprox(expected_Hj));
}

class CalculateRMSETest: public ToolsTest,
                         public ::testing::WithParamInterface<std::tuple<vector<VectorXd>, vector<VectorXd>,
						                                                             VectorXd>> {
 public:
  virtual void SetUp() {
	auto test_data = GetParam();
	estimations_ = std::get<0>(test_data);
	ground_truth_ = std::get<1>(test_data);
	expected_rmse_ = std::get<2>(test_data);
  }

 protected:
  vector<VectorXd> estimations_;
  vector<VectorXd> ground_truth_;
  VectorXd expected_rmse_;
};

TEST_P(CalculateRMSETest, ReturnCorrectRMSE_ForTestEstimations) {
  VectorXd rmse = tools_.CalculateRMSE(estimations_, ground_truth_);

  ASSERT_TRUE(rmse.isApprox(expected_rmse_));
}

INSTANTIATE_TEST_CASE_P(ToolsTest, CalculateRMSETest, ::testing::Values(
    std::make_tuple(vector<VectorXd>({Map<VectorXd>(vector<double>({1, 1, 0.2, 0.1}).data(), 4),
                                      Map<VectorXd>(vector<double>({2, 2, 0.3, 0.2}).data(), 4),
                                      Map<VectorXd>(vector<double>({3, 3, 0.4, 0.3}).data(), 4)}),
                    vector<VectorXd>({Map<VectorXd>(vector<double>({1.1, 1.1, 0.3, 0.2}).data(), 4),
                                      Map<VectorXd>(vector<double>({2.1, 2.1, 0.4, 0.3}).data(), 4),
                                      Map<VectorXd>(vector<double>({3.1, 3.1, 0.5, 0.4}).data(), 4)}),
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

TEST_P(CalculateJacobianTest, ReturnCorrectHj_ForTestState) {
  MatrixXd Hj = tools_.CalculateJacobian(state_);

  ASSERT_TRUE(Hj.isApprox(expected_Hj_, 0.0001));
}

INSTANTIATE_TEST_CASE_P(ToolsTest, CalculateJacobianTest, ::testing::Values(
    std::make_tuple(Map<VectorXd>(std::vector<double>({1, 2, 0.2, 0.4}).data(), 4),
                    Map<Matrix<double,3,4,RowMajor>>(std::vector<double>({0.447214, 0.894427, 0, 0,
                                                                          -0.4, 0.2, 0, 0,
                                                                          0, 0, 0.447214, 0.894427}).data()))));