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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;

	// create random engine	
	default_random_engine gen;

	// create a normal (Gaussian) distribution for each value
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// reserve memory for num_particles particles
	particles.reserve(num_particles);

	// initialize weigts vector with 1.
	std::fill(weights.begin(), weights.end(), 1.);

	for(int i = 0; i < num_particles; i++)
	{
		double sample_x, sample_y, sample_theta;

		// position and heading values -> sample from normal_distribution
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		// create new particle and set the sampled values and weight 1
		Particle newParticle = {};
		newParticle.id = i;
		newParticle.x = sample_x;
		newParticle.y = sample_y;
		newParticle.theta = sample_theta;
		newParticle.weight = 1.;

		// add new_particle to particles vector
		particles.push_back(newParticle);
	}

	// mark particeFilter as initialized
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// create random engine	
	default_random_engine gen;

	// for every particle
	for(int i = 0; i < num_particles; i++)
	{
		// predict next x
		double newX = particles[i].x + velocity / yaw_rate * ( sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta) );

		// predict next y
		double newY = particles[i].y + velocity / yaw_rate * ( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );

		// predict next theta (heading of vehicle)
		double newTheta = particles[i].theta + yaw_rate * delta_t;

		// add gaussian noise to the updated particle position

		// create a normal (Gaussian) distribution for each value
		normal_distribution<double> dist_x(newX, std_pos[0]);
		normal_distribution<double> dist_y(newY, std_pos[1]);
		normal_distribution<double> dist_theta(newTheta, std_pos[2]);

		// position and heading values -> sample from normal_distribution
		double sample_x, sample_y, sample_theta;
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);
		
		// assign new position and heading values
		particles[i].x = newX;
		particles[i].y = newY;
		particles[i].theta = newTheta;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// for every observed measurement
	for(int i = 0; i < observations.size(); i++)
	{
		// find the nearest predicted measurement
		int nr_nearest_predicted_measurement;
		double smallest_distance = 99999999.;

		for(int j = 0; j < predicted.size(); j++)
		{
			// calc euklidean distance
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			// is distance < known distance, then memory the measurement
			if(distance < smallest_distance)
			{
				smallest_distance = distance;
				nr_nearest_predicted_measurement = j;
			}
		}

		// assign the nearest predicted measurement to the observed measurement
		observations[i].id = nr_nearest_predicted_measurement;
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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// for each particle
	for(int i = 0; i < num_particles; i++)
	{
		// transform each map landmark into the coordinatesystem of the particle -> this will be the predicted landmarks
		std::vector<LandmarkObs> predicted;

		// reserve memory for all landmarks
		predicted.reserve(map_landmarks.landmark_list.size());

		double trans_x = particles[i].x;
		double trans_y = particles[i].y;
		double trans_angle = particles[i].theta;

		// for every map landmark calculate the coordinates in particle coordinate syytem
		for(int i = 0; i < map_landmarks.landmark_list.size(); i++)
		{
			// create a new landmark
			LandmarkObs predictedLandmark = {};
			// set id
			predictedLandmark.id = map_landmarks.landmark_list[i].id_i;

			// transform x, y
			double x_map_lm = map_landmarks.landmark_list[i].x_f;
			double y_map_lm = map_landmarks.landmark_list[i].y_f;

			// set transformed x, y
			predictedLandmark.x = x_map_lm * cos(trans_angle) - y_map_lm * sin(trans_angle) + trans_x;
			predictedLandmark.y = x_map_lm * sin(trans_angle) + y_map_lm * cos(trans_angle) + trans_y;

			// add to predicted landmarks
			predicted.push_back(predictedLandmark);
		}

		// associate observed and predicted landmarks (both in vehicle coordinate system)
		ParticleFilter::dataAssociation(predicted, & observations);
		
		// every observation landmark is now associated with the nearest predicted landmark

		// for every landmark association the probabilty of actual observation is calculated
		vector<double> probabilities_landmark_observation;

		// reserve memory
		probabilities_landmark_observation.reserve(observations.size());

		// normalizer for Multivariate-Gaussian Probability
		double norm = 1 / ( 2 * PI * std_landmark[0] * std_landmark[1]);

		// variances for Multivariate-Gaussian Probability
		double var_x = std_landmark[0] * std_landmark[0];
		double var_y = std_landmark[1] * std_landmark[1];

		for(int i = 0; i < observations.size(); i++)
		{
			// calculate the Multivariate-Gaussian Probability
			double prob = norm *
				      exp(-1 *
				        ( pow((observations[i].x - predicted[observations[i].id].x), 2) / (2 * var_x)) +
				        pow((observations[i].y - predicted[observations[i].id].y), 2) / (2 * var_y)) )
				      );

			// store observation probabilities
			probabilities_landmark_observation.push_back(prob);
		}

		// create vector for storing the weights
		vector<double> weights = {};

		// reserve memory
		weights.reserve(observations.size());

		double weight = probabilities_landmark_observation[0];
		// calculate the weight for the particle
		for(int i = 1; i < probabilities_landmark_observation.size(); i++)
		{
			weight *= probabilities_landmark_observation[i];
		}
		
		// store weight of this particle
		weights[i] = weight;
	}

	// sum of weights
	double sum_weights = 0;
	for(int i = 0; i < num_particles; i++)
	{
		sum_weights += weights[i];
	}

	// normalize weights
	for(int i = 0; i < num_particles; i++)
	{
		weights[i] = weights[i] / sum_weights;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::random_device rd;     // only used once to initialise (seed) engine
	std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
	std::uniform_int_distribution<int> uni(0, num_particles); // guaranteed unbiased

	// set random index 
	int index = uni(rng);

	// set init beta
	double beta = 0.;

	// max weight is
	double max_weight = *max_element(weights.begin(), weights.end());

	// new particles vector
	vector<Particle> newParticles;
	newParticles.resize(num_particles);

	// implementation of Resampling Wheel
	for(int i = 0; i < num_particles; i++)
	{
		beta += std::uniform_real_distribution<double> uni(0., 2*max_weight);
		
		while(weights[index] < beta)
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		
		newParticles.push_back(particles[index]);
	}

	// replace the old particles with new one
	particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
