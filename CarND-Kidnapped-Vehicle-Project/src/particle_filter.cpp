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


#define DEBUG 0

void ParticleFilter::init(double x, double y, double theta, double sigma_pos[]) {

    // set number of particles
    num_particles = 100;

    // define random engine and distributions used to add noise
    std::default_random_engine gen;
    std::normal_distribution<double> N_x_init(x, sigma_pos[0]);
    std::normal_distribution<double> N_y_init(y, sigma_pos[1]);
    std::normal_distribution<double> N_theta_init(theta, sigma_pos[2]);

    for(int i = 0; i < num_particles; i++){
        Particle p;
        p.id = i;
        p.x = N_x_init(gen);
        p.y = N_y_init(gen);
        p.theta = N_theta_init(gen);
        p.weight = 1.0; // Initialize all particles weights to 1.0
        particles.push_back(p);
    }

    is_initialized = true;
#if DEBUG == 1
    std::cout << "Init particle x: " << particles[0].x << " y: " << particles[0].y << " w: " << particles[0].weight << std::endl;
#endif

}

void ParticleFilter::prediction(double dt, double std_pos[], double v, double yaw_rate) {
    /*
     * Use motion model to predict where the car will be at next time step.
     * For each particle update particles location based on velocity and yaw rate
     * measurements.
     * Add gaussian noise to prediction
     */

    std::default_random_engine gen;

    for(int i = 0; i < num_particles; i++){

        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

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

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    // landmark Id to the observed measurement.

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
            // if the best distance between the predicted and observed measurements is greater that 1 m
            // do not assign to it the landmark id.
            if(best_dist <= 1.0)
                observations[i].id = best_id;
            else observations[i].id = -1;
        }

    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks) {
    // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

    std::default_random_engine gen;
    std::normal_distribution<double> N_x(0, std_landmark[0]);
    std::normal_distribution<double> N_y(0, std_landmark[1]);

    // weight normalization term
    double weight_norm = 0.0;

    double std_x = std_landmark[0];
    double std_xx = std_x*std_x;
    double std_y = std_landmark[1];
    double std_yy = std_y*std_y;
    double pnorm = 1.0/(2.0*M_PI*std_x*std_y);


    // iterate over each particle to find its new weight
    for(std::vector<Particle>::iterator it_prt = particles.begin(); it_prt!=particles.end(); it_prt++){
        double x = (*it_prt).x;
        double y = (*it_prt).y;
        double cos_theta = cos((*it_prt).theta);
        double sin_theta = sin((*it_prt).theta);

        // vector to store predicted landmark measurements
        std::vector<LandmarkObs> predicted;

#if DEBUG == 1
        std::cout << "* Particle (" << x << ", " << y << ") theta: " << (*it_prt).theta << std::endl;
#endif

        for(std::vector<Map::single_landmark_s>::iterator  it_landmark = map_landmarks.landmark_list.begin(); 
                it_landmark != map_landmarks.landmark_list.end(); it_landmark++){

            LandmarkObs meas;
            //Set values
            meas.id = (*it_landmark).id_i;
            // convert landmark coordinates to car coordinates, and add noise to measurement
            meas.x = (cos_theta*(*it_landmark).x_f) + (sin_theta*(*it_landmark).y_f) - (cos_theta*x) - (sin_theta*y) + N_x(gen);
            meas.y = (-sin_theta*(*it_landmark).x_f) + (cos_theta*(*it_landmark).y_f) + (sin_theta*x) - (cos_theta*y) + N_y(gen);

            double dist = sqrt((meas.x*meas.x)+(meas.y*meas.y));

            // select landmarks between range of measurement
            if(dist <= sensor_range){
                predicted.push_back(meas);
#if DEBUG == 1
                std::cout << "\tPredicted landmark in range: id[" << meas.id << "] (" << meas.x << ", " << meas.y << ")\n";
#endif
            }

        }


        //associate observations to landmarks ids.

        dataAssociation(predicted, observations);

#if DEBUG == 1
        for( int k = 0; k < observations.size(); k++){
            std::cout << "\t\tAssociated Obs: id[" << observations[k].id << "] (" << observations[k].x << ", " << observations[k].y << ")\n";

        }
#endif

        double new_weight = 1.0;
        double prob;

        for(int i = 0; i < observations.size(); i++){
            if(observations[i].id != -1){

                double obsx = cos_theta*observations[i].x - sin_theta*observations[i].y + x;
                double obsy = sin_theta*observations[i].x + cos_theta*observations[i].y + y;
                double xu = obsx - map_landmarks.landmark_list[observations[i].id-1].x_f;
                double yu = obsy - map_landmarks.landmark_list[observations[i].id-1].y_f;

                // multi-variate gaussian distribution
                prob = pnorm*exp(-((xu*xu)/(2*std_xx) + (yu*yu)/(2*std_yy)));
                new_weight *= prob;

#if DEBUG == 1
                std::cout << "\tprob: " << prob << std::endl;
#endif
            }else{
                new_weight = 0.0;
                break;
            }
        }


        (*it_prt).weight = new_weight;
        weight_norm += new_weight;

#if DEBUG == 1
        std::cout << "New weight: " << new_weight << "\n-------------------\n";
#endif
    }

    // Normalize new particles weights

    for(std::vector<Particle>::iterator it_prt = particles.begin(); it_prt!=particles.end(); it_prt++){
        (*it_prt).weight = (*it_prt).weight / weight_norm;
#if DEBUG == 1
        std::cout << "\t\t\tFinal weights: " << (*it_prt).weight << ", ";
#endif

    }

#if DEBUG == 1
    std::cout << "\n-----------\n";
#endif


}

void ParticleFilter::resample() {

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
