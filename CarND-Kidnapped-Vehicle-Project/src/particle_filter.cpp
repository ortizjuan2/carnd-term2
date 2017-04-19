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
	num_particles = 3;

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
	std::normal_distribution<double> N_x_init(x, sigma_pos[0]);
	std::normal_distribution<double> N_y_init(y, sigma_pos[1]);
	std::normal_distribution<double> N_theta_init(theta, sigma_pos[2]);

	double init_w = 1.;

	for(int i = 0; i < num_particles; i++){
		Particle p;
		p.id = i;
		p.x = N_x_init(gen);
		p.y = N_y_init(gen);
		p.theta = N_theta_init(gen);
		p.weight = init_w;
		particles.push_back(p);
	}

	is_initialized = true;

	std::cout << "Init particle x: " << particles[0].x << " y: " << particles[0].y << " w: " << particles[0].weight << std::endl;

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
	//std::normal_distribution<double> N_x(0, std_pos[0]*std_pos[0]);
	//std::normal_distribution<double> N_y(0, std_pos[1]*std_pos[1]);
	//std::normal_distribution<double> N_yaw(0, std_pos[2]*std_pos[2]);

	for(int i = 0; i < num_particles; i++){

		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		//v = v + N_v(gen);
		//yaw_rate = yaw_rate + N_yaw_rate(gen);

		if(fabs(yaw_rate) < 1e-6){
			x = x + v*dt*cos(theta);
			y = y + v*dt*sin(theta);

		}else{
			x = x + (v/yaw_rate)*(sin(theta + yaw_rate*dt) - sin(theta));
			y = y + (v/yaw_rate)*(cos(theta) - cos(theta + yaw_rate*dt));
			theta = theta + yaw_rate*dt;
		}


		// add noise
		std::normal_distribution<double> N_x(x, std_pos[0]*std_pos[0]);
		std::normal_distribution<double> N_y(y, std_pos[1]*std_pos[1]);
		std::normal_distribution<double> N_yaw(theta, std_pos[2]*std_pos[2]);


		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_yaw(gen);
	}

	std::cout << "Prediction x:" << particles[0].x << " y: " << particles[0].y << std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for(int i = 0; i < observations.size(); i++){
		double best_dist = 99999;
		double best_id = -1;

		for(int j = 0; j < predicted.size(); j++){
			double x = observations[i].x - predicted[j].x;
			double y = observations[i].y - predicted[j].y;
			double dist = sqrt(x*x + y*y);
			if(dist < best_dist){
				best_dist = dist;
				best_id	= predicted[j].id;
			}

			observations[i].id = best_id;
		}

	}

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

	
	std::default_random_engine gen;
	std::normal_distribution<double> N_x(0, std_landmark[0]);
	std::normal_distribution<double> N_y(0, std_landmark[1]);
	
	double weight_norm = 0.0;

	for(std::vector<Particle>::iterator it_prt = particles.begin(); it_prt!=particles.end(); it_prt++){
		double x = (*it_prt).x;
		double y = (*it_prt).y;
		double cos_theta = cos((*it_prt).theta);
		double sin_theta = sin((*it_prt).theta);

		std::vector<LandmarkObs> predicted;

		for(std::vector<Map::single_landmark_s>::iterator  it_landmark = map_landmarks.landmark_list.begin(); 
				it_landmark != map_landmarks.landmark_list.end(); it_landmark++){
				
				LandmarkObs meas;
				//Set values
				meas.id = (*it_landmark).id_i;
				//meas.x = cos_theta*((*it_landmark).x_f - x) + sin_theta*((*it_landmark).y_f - y) + N_x(gen);
				//meas.y = cos_theta*((*it_landmark).y_f - y) + sin_theta*((*it_landmark).x_f - x) + N_y(gen);

				meas.x = cos_theta*((*it_landmark).x_f - x) + sin_theta*((*it_landmark).y_f - y) + N_x(gen);
				meas.y = cos_theta*((*it_landmark).y_f - y) + sin_theta*((*it_landmark).x_f - x) + N_y(gen);

				double dist = sqrt((meas.x*meas.x)+(meas.y*meas.y));
					
				if(dist <= sensor_range){
						predicted.push_back(meas);
					}

				//std::cout << meas.x << " " << meas.y << std::endl;

			//std::cout << (*it_landmark).x_f << " " << (*it_landmark).y_f << std::endl;
		}


		//associate observations to landmarks ids.

		dataAssociation(predicted, observations);

		// TODO: calculate new probabilities and normalize weights

		double new_weight = 1.0;
		double std_x = std_landmark[0];
		double std_xx = std_x*std_x;
		double std_y = std_landmark[1];
		double std_yy = std_y*std_y;
		double pnorm = 1.0/(2.0*M_PI*std_x*std_y);
		double prob;

		for(int i = 0; i < observations.size(); i++){
			double x = observations[i].x - map_landmarks.landmark_list[observations[i].id-1].x_f;
			double y = observations[i].y - map_landmarks.landmark_list[observations[i].id-1].y_f;

			//if(x <= 1.0 && y <= 1.0){
				prob = pnorm*exp(-((x*x)/(2*std_xx) + (y*y)/(2*std_yy)));
				new_weight *= prob;
			//}
			std::cout << "obs: (" << observations[i].x << ", " << observations[i].y << ") landmark: (" << map_landmarks.landmark_list[observations[i].id-1].x_f << ", "
					<< map_landmarks.landmark_list[observations[i].id-1].y_f << ")\n";
			std::cout << "prob: " << prob << std::endl;

		}

		if(new_weight != 1.0){
			(*it_prt).weight = new_weight;
			weight_norm += new_weight;
		}else (*it_prt).weight = 0.0;
	}

	// Normalize

	for(std::vector<Particle>::iterator it_prt = particles.begin(); it_prt!=particles.end(); it_prt++){
		(*it_prt).weight = (*it_prt).weight / weight_norm;

		std::cout << (*it_prt).weight << std::endl;
	
	}

	std::cout << "-----------" << std::endl;


///---------------------------------------------------------


/*
	std::vector<LandmarkObs> observations_transformed;
	double x, y, distance;
	double w_norm = 0;


	for(int i=0; i<particles.size(); i++){
		// identify applicable observations and convert them to global coordinates
		for(int j=0; j<observations.size(); j++){
		x = observations[j].x;
		y = observations[j].y;
		distance = sqrt((x*x)+(y*y));
		if(distance > sensor_range) continue;

		LandmarkObs meas;
		//Set values
		meas.x = x*cos(particles[i].theta) + y*sin(particles[i].theta) + particles[i].x;
		meas.y = x*sin(particles[i].theta) + y*cos(particles[i].theta) + particles[i].y;
		observations_transformed.push_back(meas);

		}

		double best_distance;
		double best_id;
		double pxy_norm = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]);
		double sigma2x = std_landmark[0] * std_landmark[0];
		double sigma2y = std_landmark[1] * std_landmark[1];
		double p_final = 1.0;
		double pxy;
		for(int i_obs=0; i_obs<observations_transformed.size();i_obs++){
			best_distance = 999999;
			best_id = -1;
			for(int i_map=0; i_map<map_landmarks.landmark_list.size(); i_map++){
				double d1 = observations_transformed[i_obs].x - map_landmarks.landmark_list[i_map].x_f;
				double d2 = observations_transformed[i_obs].y - map_landmarks.landmark_list[i_map].y_f;
				double dist = sqrt((d1*d1) + (d2*d2));

			//	std::cout << "observation: (" << observations_transformed[i_obs].x << ", " << observations_transformed[i_obs].y << ") landmark: (";
			//	std::cout << map_landmarks.landmark_list[i_map].x_f << ", " << map_landmarks.landmark_list[i_map].y_f << ") dist: " << dist << std::endl;



				if(dist < best_distance){
					best_distance = dist;
					best_id = i_map;
				}
			}
			//std::cout << "best: " << best_distance << std::endl << "-----" << std::endl;

			if(best_id != -1){
				double xu = observations_transformed[i_obs].x - map_landmarks.landmark_list[best_id].x_f;
				double yu = observations_transformed[i_obs].y - map_landmarks.landmark_list[best_id].y_f;
				pxy = pxy_norm*exp(-(((xu*xu)/(2*sigma2x))+((yu*yu)/(2*sigma2y))));
				p_final *= pxy;

				//std::cout << "observation: (" << observations_transformed[i_obs].x << ", " << observations_transformed[i_obs].y << ") landmark: (";
					//		std::cout << map_landmarks.landmark_list[best_id].x_f << ", " << map_landmarks.landmark_list[best_id].y_f << ") dist: " << dist << std::endl;

						//	std::cout << "Probability: " << pxy << " Cumulative: " << p_final << std::endl;
			}




		}

		//std::cout << "Prob: " << p_final << std::endl;

		if(p_final != 1.0){
		//	std::cout << "add p_final" << std::endl;
				particles[i].weight = p_final;
				//w_norm += p_final;
					
		}else{
			particles[i].weight = 0.0;
		}



	}

	//weights normalization

	for(int i=0; i<particles.size(); i++){
		w_norm += particles[i].weight;
	}
	
	for(int i=0; i<particles.size(); i++){
		if(w_norm > 0.0)
			particles[i].weight /= w_norm;
	}
	//std::cout << "W Norm: " << w_norm << std::endl;
*/






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

	double weight_norm = 0;

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
		//weight_norm += particles[index].weight;

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
