#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/tools.h"

class MockTools : public Tools {
 public:
  MOCK_METHOD1(CalculateJacobian, MatrixXd(const VectorXd& x_state));

  // Use this to call CalculateJacobian() defined in Tools.
  MatrixXd ToolsCalculateJacobian(const VectorXd& x_state) { 
    return Tools::CalculateJacobian(x_state);
  }
};