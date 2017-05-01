#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;
	p_error = 0;
	d_error = 0;
	i_error = 0;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

