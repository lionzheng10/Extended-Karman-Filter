#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	//check input
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
		cout << "CalculateRMSE() - Error - Invalid input" <<endl;

	for (unsigned int i =0; i < estimations.size(); ++i)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		VectorXd residual_2 = residual.array() * residual.array();
		rmse += residual_2;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
  	float px = x_state(0);
  	float py = x_state(1);
  	float vx = x_state(2);
  	float vy = x_state(3);

	//precalculate some parameters to reduce redundant calculation
  	float c1 = px*px + py*py;
  //avoid division by zero
    if (fabs(c1)<0.0001){
      px += 0.002;
      py += 0.002;
      c1 = px*px + py*py;
      }

    float c2 = sqrt(c1);
    float c3 = c1*c2;

    

    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py-vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;

	return Hj;

}
