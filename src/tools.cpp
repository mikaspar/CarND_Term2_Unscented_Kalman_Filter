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

	rmse << 0, 0, 0, 0;

    if(estimations.empty()) {
	  cout << "No estimations delivered" << endl;
      return rmse;
    }
    if (estimations.size() != ground_truth.size())	{
	  cout << "Number of estimations does not fit the number of ground truth values" << endl;
	  return rmse;
	}

	//sum of the differences
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd difference = estimations[i] - ground_truth[i];
		difference = difference.array()*difference.array();
		rmse += difference;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}
