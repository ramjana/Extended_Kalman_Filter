#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

   VectorXd rmse(4);

   rmse << 0,0,0,0;

   //check the ground truth table size and estimation size are same
   if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
       std::cout << "Invalid estimation or ground truth data " << std::endl;
       return rmse;
   }

   for (unsigned int i =0 ; i<estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
   }
	//average by size of estimations
   rmse = rmse/estimations.size();
        // take square root
   rmse = rmse.array().sqrt();
   
   return rmse;
}

double Tools::ZeroCheck(const double &value, const double epsilon) {
 
  if(value < epsilon){ 
    return epsilon;
  }else{
    return value;
  }
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);

  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;


  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float range = Tools::ZeroCheck(sqrt(pow(px,2) + pow(py,2)));
  float range_power2 = Tools::ZeroCheck(pow(px,2) + pow(py,2));
  float range_power3_2 = Tools::ZeroCheck(sqrt(range_power2*range_power2*range_power2));
  float some_math = (vx*py - vy*px)/range_power3_2;
  
  
   //check division by zero
  if(fabs(range_power2) < 0.0001){
    std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
    return Hj;
  }
 
  // compute the Jacobian matrix

     Hj(0,0) = px/range;
     Hj(0,1) = py/range;
     Hj(1,0) = -1.*py/range_power2;
     Hj(1,1) = px/range_power2;
     Hj(2,0) = py*(some_math);
     Hj(2,1) = px*((vy*px-vx*py)/range_power3_2);
     Hj(2,2) = px/range;
     Hj(2,3) = py/range;

  return Hj;
}
