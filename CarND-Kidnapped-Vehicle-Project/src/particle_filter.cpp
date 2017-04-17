/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>


#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double sigma_pos[]) {

	// set number of particles
	num_particles = 100;

	/*
	struct Particle {

		int id;
		double x;
		double y;
		double theta;
		double weight;
	};
	*/

	//particles
	std::default_random_engine gen;
	std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
	std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
	std::normal_distribution<double> N_theta_init(0, sigma_pos[2]);

	double init_w = 1./num_particles;

	for(int i = 0; i < num_particles; i++){
		Particle p;
		p.id = i;
		p.x = x + N_x_init(gen);
		p.y = y + N_y_init(gen);
		p.theta = theta + N_theta_init(gen);
		p.weight = init_w;
		particles.push_back(p);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double dt, double std_pos[], double v, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	/*
	 * Use motion model to predict where the car will be at next time step.
	 * For each particle update particles location based on velocity and yaw rate
	 * measurements.
	 * Add gaussian noise to velocity and yaw rate.
	 */


	/*
	 * struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
};
	 *
	 */

	// TODO: Add noise ????

	std::default_random_engine gen;
	std::normal_distribution<double> N_v(0, std_pos[0]*std_pos[0]);
	std::normal_distribution<double> N_yaw_rate(0, std_pos[2]*std_pos[2]);

	for(int i = 0; i < num_particles; i++){

		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		v = v + N_v(gen);
		yaw_rate = yaw_rate + N_yaw_rate(gen);

		if(fabs(yaw_rate) < 1e-6){
			x = x + v*dt*cos(theta);
			y = y + v*dt*sin(theta);

		}else{
			x = x + (v/yaw_rate)*(sin(theta + yaw_rate*dt) - sin(theta));
			y = y + (v/yaw_rate)*(cos(theta) - cos(theta + yaw_rate*dt));
			theta = theta + yaw_rate*dt;
		}

		particles[i].x = x;
		particles[i].y = y;
		particles[i].theta = theta;
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//
	// Implementation of the algorithm described by Sebastian in its lecture
	//
	// Set of current particles
	std::vector<Particle> particles_tmp;

	std::default_random_engine int_eng{};
	std::uniform_int_distribution<> int_distribution{0, num_particles}; // type of engine

	std::default_random_engine real_eng{};
	std::uniform_real_distribution<> real_distribution{0, 1}; // type of engine


	int index = int_distribution(int_eng);


	double beta = 0.0;
	//double nw = max(weights.begin(), weights.end());


	double nw = 0;
/*
	for(std::vector<double>::iterator p = weights.begin(); p != weights.end()-1; p++){
		if (nw < (*p))
			nw = (*p);
	}
*/
	for(int i=0; i<particles.size(); i++){
		if(nw < particles[i].weight)
			nw = particles[i].weight;
	}


	for (int i=0; i<num_particles; i++){
		beta += real_distribution(real_eng) * 2.0 * nw;
		while(beta > particles[index].weight){
			beta -= particles[index].weight;
			index = (index + 1) % num_particles;
		}
		particles_tmp.push_back(particles[index]);


	}
	// set resampled particles
	particles = particles_tmp;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
