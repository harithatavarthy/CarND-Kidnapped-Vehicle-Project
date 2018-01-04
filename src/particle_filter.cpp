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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;
// declare a random engine to be used across multiple and various method calls
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    std::cout << " hello there" << std::endl;
    num_particles = 100;
    
    //define normal distribution around GPS sensor data to include sensor noise
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for (int i = 0 ; i < num_particles ; i++)
    {
        Particle P;
        P.id = i;
        //std::cout << "id:" << P.id << std::endl;
        P.x = dist_x(gen);
        //std::cout << "x:" << P.x << std::endl;
        P.y = dist_y(gen);
        //std::cout << "y:" << P.y << std::endl;
        P.theta = dist_theta(gen);
        //std::cout << "theta:" << P.theta << std::endl;
        P.weight = 1.0;
        particles.push_back(P);
        
    }
    
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    //define normal distributions for sensor noise with zero mean for position information.
    normal_distribution<double> pred_x(0,std_pos[0]);
    normal_distribution<double> pred_y(0,std_pos[1]);
    normal_distribution<double> pred_theta(0,std_pos[2]);
    
    //calculate the new state of each particle using prior observations velocatity and Yaw Rate
    for (int i = 0 ; i < num_particles ; i++)
    {
        if(fabs(yaw_rate) < 0.00001)
        {
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        }
        else
        {
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
            
        }
        
        //define normal distributions for sensor noise with zero mean for position information.
        //normal_distribution<double> pred_x(particles[i].x,std_pos[0]);
        //normal_distribution<double> pred_y(particles[i].y,std_pos[1]);
        //normal_distribution<double> pred_theta(particles[i].theta,std_pos[2]);
        
        //add noise
        particles[i].x += pred_x(gen);
        particles[i].y += pred_y(gen);
        particles[i].theta += pred_theta(gen);
        

    }
    
    

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    
    //Assumption -- Sensor Observations passed to this function have been transformed to map coordinate
    //system from vehicle coordinate system.
    
   //for each sensor observation of landmark position ....
   for (unsigned int i = 0 ; i < observations.size(); i++ ){
       double minimum_dist = numeric_limits<double>::max();
       int temp_id = -1;
       //int index = -1;
        LandmarkObs LO = observations[i];
       //Identify the particle closest to the landmark and associate the particle to that landmark.
        for(unsigned int j = 0 ; j < predicted.size(); j++){
            LandmarkObs LP = predicted[j];
            double est_dist = dist(LO.x,LO.y,LP.x,LP.y);
            if(est_dist < minimum_dist){
                minimum_dist = est_dist;
                temp_id = LP.id;
            }
            
        }
       observations[i].id = temp_id;
   
    }
  
}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    // for each particle...
    for (unsigned int i = 0 ; i < num_particles ; i++){
        // fetch and save the particles position and heading direction
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        
        // vector to hold landmark locations within the sensor range of particles.
        std::vector<LandmarkObs> predictions;
        
        //for each landmark.....
        for(unsigned int j=0; j < map_landmarks.landmark_list.size(); j++){
            double lm_x = map_landmarks.landmark_list[j].x_f;
            double lm_y = map_landmarks.landmark_list[j].y_f;
            int lm_id = map_landmarks.landmark_list[j].id_i;
            
            //we will consider only those landmarks that are withing the sensor range distance of the particle we are processing. By placing the particle at the center, we will look for any observation that is within the radius equal to sensor range..
            //if (fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range){
            if(fabs(dist(lm_x,lm_y,p_x,p_y)) <= sensor_range){
                //std::cout << "lm_x - P_x : " << fabs(lm_x -p_x) << std::endl;
                //std::cout << "lm_y - P_y : " << fabs(lm_y -p_y) << std::endl;
                //int clc = sqrt((lm_x - p_x)*(lm_x - p_x) + (lm_y - p_y)*(lm_y - p_y));
                //std::cout << "lcal: " << clc << std::endl;
                //std::cout << "dist(lm_x,lm_y,p_x,p_y) : " << fabs(dist(lm_x,lm_y,p_x,p_y)) << std::endl;
                //std::cout << "sensor_range : " << sensor_range << std::endl;
                

                predictions.push_back(LandmarkObs{lm_id,lm_x,lm_y});
            }
            
        }
        
        //will now transform the observations to landmarks (from vehicles sensors ) from vehicle coordinate
        //system to map coordinate system. We will do this for all the observations and from each particles
        //perspective.
        std::vector<LandmarkObs> transformed_obs;
        for(unsigned int k = 0 ; k < observations.size() ; k++){
            double t_x = cos(p_theta) * observations[k].x - sin(p_theta) * observations[k].y + p_x;
            double t_y = sin(p_theta) * observations[k].x + cos(p_theta) * observations[k].y + p_y;
            transformed_obs.push_back(LandmarkObs{observations[k].id,t_x,t_y});
            
        }
        
        //perform data association -- To associate nearest landmark to each of the particle
        dataAssociation(predictions,transformed_obs);
        
        // reinit weight
        particles[i].weight = 1.0;
        
        //Now we will use assign weights to our particles... In order to assign a weight, we will first go through our list of transformed observations to landmarks within the sensor range distance of our particle, identify the landmark associated to the particle and use multi variate guassian technique to assign weights
        double temp_x, temp_y ;
        for(unsigned int l = 0 ; l < transformed_obs.size() ; l++) {
            for (unsigned int m = 0; m < predictions.size() ; m++) {
                if(predictions[m].id == transformed_obs[l].id){
                    temp_x = predictions[m].x;
                    temp_y = predictions[m].y;
                }
                
            }
            //calculate weight for this particle using multi-variate guassian
            double sig_x = std_landmark[0];
            double sig_y = std_landmark[1];
            double p_weight = ( 1/(2* M_PI * sig_x * sig_y)) * exp( -(pow(temp_x - transformed_obs[l].x,2)/(2 * pow(sig_x, 2)) + (pow(temp_y - transformed_obs[l].y,2)/(2 * pow(sig_y, 2))) ) );
            
            particles[i].weight *= p_weight;
        }
        
    }
}
 



void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    vector<Particle> new_particles;
    
    // obtain all of the current weights
    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }
    
    // generate random starting index for resampling wheel
    uniform_int_distribution<int> uniintdist(0, num_particles-1);
    auto index = uniintdist(gen);
    
    // get max weight
    double max_weight = *max_element(weights.begin(), weights.end());
    
    // uniform random distribution [0.0, max_weight)
    uniform_real_distribution<double> unirealdist(0.0, max_weight);
    
    double beta = 0.0;
    
    // spin the resample wheel!
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    
    particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
