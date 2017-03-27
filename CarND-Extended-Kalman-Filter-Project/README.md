# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

[//]: # (Image References)

[image1]: ./img/kf_generalization.jpg
[image2]: ./img/ekf_result.png


## EKF Algorithm Generalization
Below if a comparison for reference of the Kalman Filter when dealing with linear functions and the Extended Kalman Filter used when dealing with non-linear functions.

Please note that in this project we are using a linear model for the prediction step, so in this case although we are implementing the EKF algorithm, it is no necessary to replace `F` with `Fj` (F jacobian). 

The measurement update for lidar will also use the Kalman Filter equations, since lidar uses linear equations. In the case of the Radar, we will need to replace `H` with `Hj` (H jacobian) becuase Radar uses non-linear equations.

![alt text][image1]

## C++ Implementation
The source code for this project is located in the `src` folder. There is a `main.cpp` file with that controls the main flow of the program and also the following files:
`FusionEKF.cpp`: In charge of initializing the state and matrices of the algorithm. Also controls the flow of the perdiction and update steps.
`kalman_filter.cpp`: This is a class used to implement the algorithm calculations. It has the prediction and updates (linear and non-linear) routines. They are called from the `FusinEKF.cpp` file.

## Program Flow:
1). The measuremennt processor/matlab simulator is generating the FUSION .txt file:
  `./data/sample-laser-radar-measurement-data-1.txt`
  OR
   `./data/sample-laser-radar-measurement-data-1.txt`
  
   The Input file format is:
  `L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy`
  `R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy`
 
  Example:
  `R   8.60363 0.0290616   -2.99903    1477010443399637    8.6 0.25    -3.00029    0`
  ` L   8.45    0.25    1477010443349642    8.45    0.25    -3.00027    0`

2). The EKF Algorithm reads form file reads all the lines and generates measurement structures

 Below is the code inside the `main.cpp` file used to parse the input file.
 
```cpp
 while (getline(in_file_, line)) {

    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
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
      // add measurement to list
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
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
      // add measurement to list
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    // it is idependent of type of sensor measurement
    // so read the rest of the line
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    // add gt vector to list
    gt_pack_list.push_back(gt_package);
  }
```
3). The ProcessMeasurement() is called with individual measurements (one by one), and the resulting estimate is stored in the `output.txt` file.

Output file format:
est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy

```cpp
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";
	.
	.
	.

```
4). Inside the `ProcessMeasurement` routine the first step performed is to initialize the stimates and matrices. Then for the next readings the routine check if is a Radar or Lidar measurement to use the correct Update routine. (Linear or non-linear)
 
```cpp
 
 void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	//Initialization
  if (!is_initialized_) {
    // first measurement
    ekf_.x_ = VectorXd(4);
	float py, px;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//      Convert radar from polar to cartesian coordinates and initialize state.
 
    	px = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
    	py = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
  
  //    Initialize state.
  
    	px = measurement_pack.raw_measurements_[0];
    	py = measurement_pack.raw_measurements_[1];
    }

    ekf_.x_ << px, py, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
 
   //  Prediction

   	const float noise_ax = 9;
  	const float noise_ay = 9;

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
		float dt_3 = dt_2 * dt;
		float dt_4 = dt_3 * dt;

		//Modify the F matrix so that the time is integrated
		ekf_.F_(0, 2) = dt;
		ekf_.F_(1, 3) = dt;

		//set the process covariance matrix Q
		ekf_.Q_ <<  (dt_4/4)*noise_ax, 0, (dt_3/2)*noise_ax, 0,
				   0, (dt_4/4)*noise_ay, 0, (dt_3/2)*noise_ay,
				   (dt_3/2)*noise_ax, 0, dt_2*noise_ax, 0,
				   0, (dt_3/2)*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

   //  Update
   
   if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);

  }
}

```
5). The predict and updates routines are inside the `kalman_filter.cpp` file.

The predict step just update the state using the lienar `F` matrix.
```cpp
void KalmanFilter::Predict() {
  	x_ = F_*x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}
```

6). The linear update routine is used to update the state when receiving Lidar measurements.

```cpp
void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

```
7). The non-linear update routine is used to update the state when receiving Radar measurements.

```cpp
void KalmanFilter::UpdateEKF(const VectorXd &z) {
		float rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
		float phi = atan2(x_[1], x_[0]);
		float rho_dot;

		if(rho != 0.0){
			rho_dot = (x_[0]*x_[2] + x_[1]*x_[3])/rho;
		}else{
			rho_dot = 0;
		}

		VectorXd z_pred(3);
		z_pred << rho, phi, rho_dot;
		VectorXd y = z - z_pred;
		MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ * Ht + R_;
		MatrixXd Si = S.inverse();
		MatrixXd PHt = P_ * Ht;
		MatrixXd K = PHt * Si;

		//new estimate
		x_ = x_ + (K * y);
		long x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;

}

```

8). The following routine is inside the `tools.cpp` file. It is used to calculate the Jacobian matrix `Hj` used in the non-linear update. 


```cpp
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	Hj << 0, 0, 0, 0,
		  0, 0, 0, 0,
		  0, 0, 0, 0;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = sqrt(c1*c1*c1);

	//check division by zero
	if(fabs(c1) < 0.0001){
		std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;

}

```

##Final Results after implementing the algorithm:
The following image shows the final results of the implementation. Here you can see that the algorith works quite well in the sections that are more linear but not so well on the curves. But in general it has very good results.

The accuracy on the first sample was:
```
Accuracy - RMSE:
0.0651649
0.0605378
  0.54319
 0.544191
```

The accuracy on the second sample was:
```
Accuracy - RMSE:
0.185496
0.190302
0.476754
0.804469
```

![alt text][image2]
