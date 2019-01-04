#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/tools.h"

class MockTools : public Tools {
 public:
  MOCK_METHOD1(CalculateJacobian, Eigen::MatrixXd(const Eigen::VectorXd& x_state));

  // Use this to call CalculateJacobian() defined in Tools.
  Eigen::MatrixXd ToolsCalculateJacobian(const Eigen::VectorXd& x_state) {
    return Tools::CalculateJacobian(x_state);
  }
};