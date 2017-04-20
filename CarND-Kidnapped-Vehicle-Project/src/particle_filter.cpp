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
#if DEBUG == 1
    std::cout << "Init particle x: " << particles[0].x << " y: " << particles[0].y << " w: " << particles[0].weight << std::endl;
#endif

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

    //std::cout << "Prediction x:" << particles[0].x << " y: " << particles[0].y << std::endl;

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
            if(best_dist <= 1.0)
                observations[i].id = best_id;
            else observations[i].id = -1;
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

#if DEBUG == 1
        std::cout << "* Particle (" << x << ", " << y << ") theta: " << (*it_prt).theta << std::endl;
#endif

        for(std::vector<Map::single_landmark_s>::iterator  it_landmark = map_landmarks.landmark_list.begin(); 
                it_landmark != map_landmarks.landmark_list.end(); it_landmark++){

            LandmarkObs meas;
            //Set values
            meas.id = (*it_landmark).id_i;
            //meas.x = cos_theta*((*it_landmark).x_f - x) + sin_theta*((*it_landmark).y_f - y) + N_x(gen);
            //meas.y = cos_theta*((*it_landmark).y_f - y) + sin_theta*((*it_landmark).x_f - x) + N_y(gen);

            meas.x = (cos_theta*(*it_landmark).x_f) + (sin_theta*(*it_landmark).y_f) - (cos_theta*x) - (sin_theta*y) + N_x(gen);
            meas.y = (-sin_theta*(*it_landmark).x_f) + (cos_theta*(*it_landmark).y_f) + (sin_theta*x) - (cos_theta*y) + N_y(gen);

            double dist = sqrt((meas.x*meas.x)+(meas.y*meas.y));

            if(dist <= sensor_range){
                predicted.push_back(meas);
#if DEBUG == 1
                std::cout << "\tPredicted landmark in range: id[" << meas.id << "] (" << meas.x << ", " << meas.y << ")\n";
#endif
            }

            //std::cout << meas.x << " " << meas.y << std::endl;

            //std::cout << (*it_landmark).x_f << " " << (*it_landmark).y_f << std::endl;
        }


        //associate observations to landmarks ids.

        dataAssociation(predicted, observations);
#if DEBUG == 1
        for( int k = 0; k < observations.size(); k++){
            std::cout << "\t\tAssociated Obs: id[" << observations[k].id << "] (" << observations[k].x << ", " << observations[k].y << ")\n";

        }
#endif

        double new_weight = 1.0;
        double std_x = std_landmark[0];
        double std_xx = std_x*std_x;
        double std_y = std_landmark[1];
        double std_yy = std_y*std_y;
        double pnorm = 1.0/(2.0*M_PI*std_x*std_y);
        double prob;

        for(int i = 0; i < observations.size(); i++){
            if(observations[i].id != -1){

                double obsx = cos_theta*observations[i].x - sin_theta*observations[i].y + x;
                double obsy = sin_theta*observations[i].x + cos_theta*observations[i].y + y;
                double xu = obsx - map_landmarks.landmark_list[observations[i].id-1].x_f;
                double yu = obsy - map_landmarks.landmark_list[observations[i].id-1].y_f;


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

    // Normalize

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
