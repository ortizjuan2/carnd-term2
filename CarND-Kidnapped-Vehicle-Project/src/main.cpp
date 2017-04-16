/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include <fstream>
#include <sstream>


#define DEBUG 0


#include "particle_filter.h"
#include "helper_functions.h"



using namespace std;



int main() {
	
	// parameters related to grading.
	int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	double max_runtime = 45; // Max allowable runtime to pass [sec]
	double max_translation_error = 1; // Max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // Max allowable yaw error [rad]

	// Start timer.
	double start = clock();
	
	//Set up parameters here
	double delta_t = 0.1; // Time elapsed between measurements [sec]
	double sensor_range = 50; // Sensor range [m]
	
	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
	double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> N_x_init(0, sigma_pos[0]);
	normal_distribution<double> N_y_init(0, sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;
	// Read map data
	/*
	 *  92.064	-34.777	1
		61.109	-47.132	2
		17.42	-4.5993	3
	 *
	 * // Set values
		single_landmark_temp.id_i = id_i;
		single_landmark_temp.x_f  = landmark_x_f;
		single_landmark_temp.y_f  = landmark_y_f;

		// Add to landmark list of map:
		map.landmark_list.push_back(single_landmark_temp);
	 *
	 */
	Map map;
	if (!read_map_data("../data/map_data.txt", map)) {
		cout << "Error: Could not open map file" << endl;
		return -1;
	}

	// Read position data
	/*
	 *
	 * 	3.9611 3.0937
		4.0378 -0.0081013
		4.1173 0.0055507
		4.2066 0.011087
		4.3007 -0.0010223
		4.4045 -0.0090234
	 *
	 *
	 * 	// Set values
		meas.velocity = velocity;
		meas.yawrate = yawrate;

		// Add to list of control measurements:
		position_meas.push_back(meas);

	 */
	vector<control_s> position_meas;
	if (!read_control_data("../data/control_data.txt", position_meas)) {
		cout << "Error: Could not open position/control measurement file" << endl;
		return -1;
	}
	
	// Read ground truth data
	/*
	 * 	6.2785 1.9598 0
		6.6632 2.0825 0.30937
		7.0554 2.2076 0.30856
		7.4561 2.3358 0.30912
		7.8656 2.4671 0.31022
		8.2852 2.6011 0.31012
		8.7144 2.7375 0.30922
		9.1536 2.8763 0.30781
	 *
	 *
	 * // Set values
		single_gt.x = x;
		single_gt.y = y;
		single_gt.theta = azimuth;

		// Add to list of control measurements and ground truth:
		gt.push_back(single_gt);
	 *
	 *
	 */
	vector<ground_truth> gt;
	if (!read_gt_data("../data/gt_data.txt", gt)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}
	
	// Run particle filter!
	int num_time_steps = position_meas.size();
	ParticleFilter pf;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	
	/*
	 *  observations for current time step
	 *  this is supoused to be the distance from the car to visible lanmarks
	 *  in car coordenates
	 *
	 * 	2.4853 5.6048
		11.142 -6.5591
		-19.92 -2.0582
		1.9234 -22.93
		13.861 -22.11
		29.925 12.867
		-13.407 -36.5
		30.348 -30.858
		-36.114 -25.237
		22.62 -41.714
		-46.064 -14.575
	 *
	 *
	 * // Set values
		meas.x = local_x;
		meas.y = local_y;

		// Add to list of control measurements:
		observations.push_back(meas);

	 */

	for (int i = 0; i < num_time_steps; ++i) {
		cout << "Time step: " << i << endl;
		// Read in landmark observations for current time step.
		ostringstream file;
		file << "../data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
		vector<LandmarkObs> observations;
		if (!read_landmark_data(file.str(), observations)) {
			cout << "Error: Could not open observation file " << i+1 << endl;
			return -1;
		}
		
		// Initialize particle filter if this is the first time step.
		if (!pf.initialized()) {
			n_x = N_x_init(gen);
			n_y = N_y_init(gen);
			n_theta = N_theta_init(gen);
			pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);

#if DEBUG == 1
			/*
			 * Output x, y position of particles to disk
			 */
			string out_file_name_ = "output.txt";
			  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

			  out_file_ << gt[i].x << "," << gt[i].y << "\n";

			  for(int i=0; i<pf.particles.size(); i++){
				out_file_ << pf.particles[i].x << "," << pf.particles[i].y << "\n";
			  }

			  if (out_file_.is_open()) {
			      out_file_.close();
			    }
#endif

		}
		else {
			// Predict the vehicle's next state (noiseless).
			pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
		}
		// simulate the addition of noise to noiseless observation data.
		vector<LandmarkObs> noisy_observations;
		LandmarkObs obs;
		for (int j = 0; j < observations.size(); ++j) {
			n_x = N_obs_x(gen);
			n_y = N_obs_y(gen);
			obs = observations[j];
			obs.x = obs.x + n_x;
			obs.y = obs.y + n_y;
			noisy_observations.push_back(obs);
		}

		// Update the weights and re-sample
		pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		pf.resample();
		
		// Calculate and output the average weighted error of the particle filter over all time steps so far.
		vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;
		for (int i = 0; i < num_particles; ++i) {
			if (particles[i].weight > highest_weight) {
				highest_weight = particles[i].weight;
				best_particle = particles[i];
			}
		}
		double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);

		for (int j = 0; j < 3; ++j) {
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}
		
		// Print the cumulative weighted error
		cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;
		
		// If the error is too high, say so and then exit.
		if (i >= time_steps_before_lock_required) {
			if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error) {
				if (cum_mean_error[0] > max_translation_error) {
					cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else if (cum_mean_error[1] > max_translation_error) {
					cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else {
					cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
				}
				return -1;
			}
		}
	}
	
	// Output the runtime for the filter.
	double stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	cout << "Runtime (sec): " << runtime << endl;
	
	// Print success if accuracy and runtime are sufficient (and this isn't just the starter code).
	if (runtime < max_runtime && pf.initialized()) {
		cout << "Success! Your particle filter passed!" << endl;
	}
	else if (!pf.initialized()) {
		cout << "This is the starter code. You haven't initialized your filter." << endl;
	}
	else {
		cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
		return -1;
	}
	
	return 0;
}


