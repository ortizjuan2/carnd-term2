#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	    rmse << 0,0,0,0;

	    // check the validity of the following inputs:
	    //  * the estimation vector size should not be zero
	    //  * the estimation vector size should equal ground truth vector size
	    if(estimations.size() != ground_truth.size()
	            || estimations.size() == 0){
	        std::cout << "Invalid estimation or ground_truth data" << std::endl;
	        return rmse;
	    }

	    //accumulate squared residuals
	    for(unsigned int i=0; i < estimations.size(); ++i){

	        VectorXd residual = estimations[i] - ground_truth[i];

	        //coefficient-wise multiplication
	        residual = residual.array()*residual.array();

	        //residual = residual.array().sqrt();
	        rmse += residual;

	        //std::cout << i << std::endl << estimations[i] << std::endl << "-------" << std::endl << ground_truth[i] << std::endl << "--------" << std::endl;
	        //std::cout << i << " " << rmse << std::endl << "....." << std::endl;


	    }

	    //calculate the mean
	    rmse = rmse/estimations.size();

	    //calculate the squared root
	    rmse = rmse.array().sqrt();

	    //return the result
	    return rmse;
}
