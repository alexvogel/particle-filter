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

void ParticleFilter::printParticles(char step){
	
	for(int i = 0; i < num_particles; i++)
	{
		std::cout << step << ": particle " << particles[i].id << ", (" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << "), w=" << particles[i].weight << std::endl;
		for(int j = 0; j < particles[i].associations.size(); j++)
		{
			std::cout << particles[i].associations[j] << ", (" << particles[i].sense_x[j] << ", " << particles[i].sense_y[j] << ")";
		}
		std::cout << std::endl;
	}
}

void ParticleFilter::transformLandmarkMap2Vehicle(double vehicle_x, double vehicle_y, double vehicle_theta, double lm_map_x, double lm_map_y, double & lm_veh_x, double & lm_veh_y ){
	// transforms a landmark from map coordinates to vehicle coordinates
	// Inputs:
	// vehicle_x: vehicle x-coordinate in map coordinate system
	// vehicle_y: vehicle y-coordinate in map coordinate system
	// vehicle_theta: vehicle heading, angle in radians counterclockwise from x-axis of map coordinate system
	// lm_map_x: landmark x-coordinate in map coordinate system
	// lm_map_y: landmark x-coordinate in map coordinate system
	// lm_veh_x: landmark x-coordinate in vehicle coordinate system. this value gets overidden
	// lm_veh_y: landmark x-coordinate in vehicle coordinate system. this value gets overidden

	// setting transformation parameter
	double trans_x = -1 * vehicle_x;
	double trans_y = -1 * vehicle_y;
	double trans_angle = -1 * vehicle_theta;

	// set transformed x, y
	lm_veh_x = (lm_map_x + trans_x) * cos(trans_angle) - (lm_map_y + trans_y) * sin(trans_angle);
	lm_veh_y = (lm_map_x + trans_x) * sin(trans_angle) + (lm_map_y + trans_y) * cos(trans_angle);
}

void ParticleFilter::transformLandmarkVehicle2Map(double vehicle_x, double vehicle_y, double vehicle_theta, double & lm_map_x, double & lm_map_y, double lm_veh_x, double lm_veh_y ){
	// transforms a landmark from map coordinates to vehicle coordinates
	// Inputs:
	// vehicle_x: vehicle x-coordinate in map coordinate system
	// vehicle_y: vehicle y-coordinate in map coordinate system
	// vehicle_theta: vehicle heading, angle in radians counterclockwise from x-axis of map coordinate system
	// lm_map_x: landmark x-coordinate in map coordinate system. this value gets overidden
	// lm_map_y: landmark x-coordinate in map coordinate system. this value gets overidden
	// lm_veh_x: landmark x-coordinate in vehicle coordinate system. 
	// lm_veh_y: landmark x-coordinate in vehicle coordinate system.

	// setting transformation parameter
	double trans_x = vehicle_x;
	double trans_y = vehicle_y;
	double trans_angle = vehicle_theta;

	// set transformed x, y
	lm_map_x = lm_veh_x * cos(trans_angle) - lm_veh_y * sin(trans_angle) + trans_x;
	lm_map_y = lm_veh_x * sin(trans_angle) + lm_veh_y * cos(trans_angle) + trans_y;
}


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// debug
	//std::cout << "- calling init" << std::endl;
	
	num_particles = 15;

	// create random engine	
	default_random_engine gen;

	// create a normal (Gaussian) distribution for each value
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// reserve memory for num_particles particles and weights
	particles.reserve(num_particles);

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

	// debug
	//printParticles('I');
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// debug
	//std::cout << "- calling prediction" << std::endl;

	// create random engine	
	default_random_engine gen;

	// debug
	//printParticles('p');

	// for every particle
	for(int i = 0; i < num_particles; i++)
	{
		//debug
		//std::cout << "prediction: processing particle: " << i << particles[i].id << std::endl;

		// predict x, y and theta
		double newX, newY, newTheta;

		// predict next x if yaw_rate == zero
		if(yaw_rate == 0)
		{
			newX = particles[i].x + velocity * delta_t * cos(particles[i].theta);

			// predict next y
			newY = particles[i].y + velocity * delta_t * sin(particles[i].theta);

			// predict next theta (heading of vehicle)
			newTheta = particles[i].theta;
		}
		// predict next x if yaw_rate != zero
		else
		{
			newX = particles[i].x + (velocity / yaw_rate) * ( sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta) );

			// predict next y
			newY = particles[i].y + (velocity / yaw_rate) * ( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );

			// predict next theta (heading of vehicle)
			newTheta = particles[i].theta + yaw_rate * delta_t;
		}
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
		particles[i].x = sample_x;
		particles[i].y = sample_y;
		particles[i].theta = sample_theta;
	}

	// debug
	//printParticles('P');
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, int id_particle ) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// debug
	//std::cout << "- calling dataAssociation: " << observations.size() << " observations " << predicted.size() << " predictions" << std::endl;

	// create vectors for setting the associations
	vector<int> associations;
	vector<double> sense_x;
	vector<double> sense_y;
		
	// for every observed measurement
	for(int i = 0; i < observations.size(); i++)
	{

		// find the nearest predicted measurement
		int nr_nearest_predicted_measurement;
		double smallest_distance = 99999999.;
		LandmarkObs nearest_prediction;

		for(int j = 0; j < predicted.size(); j++)
		{
			// calc euklidean distance
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			// is distance < known distance, then memory the measurement
			if(distance < smallest_distance)
			{
				smallest_distance = distance;
				nr_nearest_predicted_measurement = j;
				nearest_prediction = predicted[j];
			}
		}

		// assign the nearest predicted measurement to the observed measurement
		observations[i].id = nr_nearest_predicted_measurement;
		
		// set association data in particle
		associations.push_back(nearest_prediction.id);
		
		// transform landmark coordinates from vehicle coordinate system to map coordinate system
		double lm_map_x;
		double lm_map_y;		
		transformLandmarkVehicle2Map(particles[id_particle].x,
						particles[id_particle].y,
						particles[id_particle].theta,
						lm_map_x,
						lm_map_y,
						nearest_prediction.x,
						nearest_prediction.y);

		// debug - check transformations
		//std::cout << "NEAREST LANDMARK FOUND: nr " << nearest_prediction.id << std::endl;
		//std::cout << "associations vector length is now " << associations.size() << std::endl;
		//double check_lm_veh_x;
		//double check_lm_veh_y;
		//transformLandmarkMap2Vehicle(particles[id_particle].x,
		//				particles[id_particle].y,
		//				particles[id_particle].theta,
		//				lm_map_x,
		//				lm_map_y,
		//				check_lm_veh_x,
		//				check_lm_veh_y);
		//std::cout << "landmark in vehicle coordinate system BEFORE transformation: (" << nearest_prediction.x << ", " << nearest_prediction.y << ")" << std::endl;
		//std::cout << "landmark in vehicle coordinate system AFTER transformation:  (" << check_lm_veh_x << ", " << check_lm_veh_y << ")" << std::endl;

		// set sense data in particle
		sense_x.push_back(lm_map_x);
		sense_y.push_back(lm_map_y);

		// debug
		//sense_x.push_back(10. * associations.size());
		//sense_y.push_back(5. * associations.size() );

		//std::cout << "nearest distance " << smallest_distance << std::endl;
		//std::cout << "observation (" << observations[i].x << ", " << observations[i].y << ")" << std::endl;
		//std::cout << "prediction (" << nearest_prediction.x << ", " << nearest_prediction.y << ")" << std::endl;

	}

	// debug
	//std::cout << "associations for this particle are ";
	//for(int i = 0; i < associations.size(); i++)
	//{
	//	std::cout << associations[i] << ", ";
	//}
	//std::cout << std::endl;
	

	// association in the particles
	particles[id_particle] = SetAssociations(particles[id_particle], associations, sense_x, sense_y);

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

	// debug
	//std::cout << "- calling updateWeights: sensor_range=" << sensor_range << std::endl;

	// Vector of weights of all particles
	vector<double> weights;
	weights.reserve(num_particles);

	// for each particle
	for(int i = 0; i < num_particles; i++)
	{
		// transform each map landmark into the coordinatesystem of the particle -> this will be the predicted landmarks
		std::vector<LandmarkObs> predicted;

		// reserve memory for all landmarks
		predicted.reserve(map_landmarks.landmark_list.size());

		//double trans_x = -1 * particles[i].x;
		//double trans_y = -1 * particles[i].y;
		//double trans_angle = -1 * particles[i].theta;

		// for every map landmark calculate the coordinates in particle coordinate system
		for(int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			// create a new landmark
			LandmarkObs predictedLandmark = {};
			// set id
			predictedLandmark.id = map_landmarks.landmark_list[j].id_i;
			
			// debug
			//std::cout << "landmark id " << map_landmarks.landmark_list[j].id_i << std::endl;

			transformLandmarkMap2Vehicle(particles[i].x,
							particles[i].y, 
							particles[i].theta,
							map_landmarks.landmark_list[j].x_f,
							map_landmarks.landmark_list[j].y_f,
							predictedLandmark.x,
							predictedLandmark.y );

			// add to predicted landmarks if distance < sensor_range
			double distance_predicted_landmark = dist(0., 0., predictedLandmark.x, predictedLandmark.y);
			if( distance_predicted_landmark < sensor_range)
			{
				predicted.push_back(predictedLandmark);
				// debug
				//std::cout << "adding landmark id " << predictedLandmark.id << std::endl;
				//std::cout << "distance of predicted landmark = " << distance_predicted_landmark << std::endl;
				//std::cout << "updateWeights: particle " << particles[i].id << ": map landmark " << map_landmarks.landmark_list[j].id_i << " x=" << map_landmarks.landmark_list[j].x_f << " y=" << map_landmarks.landmark_list[i].y_f << std::endl;
				//std::cout << "updateWeights: particle " << particles[i].id << ": pred landmark " << predictedLandmark.id << " x=" << predictedLandmark.x << " y=" << predictedLandmark.y << std::endl;
			}

		}

		// associate observed and predicted landmarks (both in vehicle coordinate system)
		// the landmark id in the observations will be set to the nearest map landmark id
		ParticleFilter::dataAssociation(predicted, observations, i);
		
		// every observation landmark is now associated with the nearest predicted landmark

		// for every landmark association the probabilty of actual observation is calculated
		vector<double> probabilities_landmark_observation;
		probabilities_landmark_observation.reserve(observations.size());

		// normalizer for Multivariate-Gaussian Probability
		double norm = 1 / ( 2 * M_PI * std_landmark[0] * std_landmark[1]);

		// variances for Multivariate-Gaussian Probability
		double var_x = std_landmark[0] * std_landmark[0];
		double var_y = std_landmark[1] * std_landmark[1];

		for(int i = 0; i < observations.size(); i++)
		{
			// calculate the Multivariate-Gaussian Probability
			double prob = norm *
				      exp(-1 *
				        (
						pow((observations[i].x - predicted[observations[i].id].x), 2) / (2 * var_x) +
				        	pow((observations[i].y - predicted[observations[i].id].y), 2) / (2 * var_y)
					)
				      );

			// debug
			//std::cout << "probability of this landmark being detected: " << prob << std::endl;

			// probability being zero prevention
			if(prob < 1.E-15)
			{
				prob = 1.E-15;
				
				// debug
				//std::cout << "setting prob of landmarkl detection to 1.E-15" << std::endl;
			}

			// store observation probabilities
			probabilities_landmark_observation.push_back(prob);
			//debug
			//std::cout << "prob of observation " << observations[i].id << " = " << prob << std::endl;
		}

		// multiply prob of all landmarks
		double weight_obs = probabilities_landmark_observation[0];
		// calculate the weight for the particle
		
		for(int i = 1; i < probabilities_landmark_observation.size(); i++)
		{
			weight_obs *= probabilities_landmark_observation[i];
		}
		
		// store weight of this particle
		weights[i] = weight_obs;
	}

	// sum of weights
	double sum_weights = 0;
	for(int i = 0; i < num_particles; i++)
	{
		sum_weights += weights[i];
	}

	// debug	
	//std::cout << "sum of weights (from weight vector) BEFORE normalizing: " << sum_weights << std::endl;

	// normalize weights and update in particles
	for(int i = 0; i < num_particles; i++)
	{
		weights[i] = weights[i] / sum_weights;
		particles[i].weight = weights[i];
	}

	// debug
	//double sum_weights2 = 0;
	//for(int i = 0; i < num_particles; i++)
	//{
	//	sum_weights2 += weights[i];
	//}
	//std::cout << "sum of weights (from weight vector) AFTER normalizing: " << sum_weights2 << std::endl;
	//double sum_weights3 = 0;
	//for(int i = 0; i < num_particles; i++)
	//{
	//	sum_weights3 += particles[i].weight;
	//}
	//std::cout << "sum of weights (from particles) AFTER normalizing: " << sum_weights3 << std::endl;
	//if( (sum_weights2 < 0.99) || (sum_weights2 > 1.01) || isnan(sum_weights2))
	//{
	//	std::cout << "ERROR IN NORMALIZING" << std::endl;
	//	exit(1);
	//}
	//if( (sum_weights3 < 0.99) || (sum_weights3 > 1.01) || isnan(sum_weights3) )
	//{
	//	std::cout << "ERROR IN NORMALIZING" << std::endl;
	//	exit(1);
	//}

	// debug
	//printParticles('W');
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// debug
	//std::cout << "- calling resample" << std::endl;
	//printParticles('r');

	// instead of default_random_engine
	std::random_device rd;     // only used once to initialise (seed) engine
	std::mt19937 gen(rd());    // random-number engine used (Mersenne-Twister in this case)
	std::uniform_int_distribution<int> dist_int(0, num_particles-1); // guaranteed unbiased

	// set random index 
	int index = dist_int(gen);

	// set init beta
	double beta = 0.;

	// Vector of weights of all particles
	vector<double> weights;
	weights.reserve(num_particles);

	double max_weight = 0;

	for(int i = 0; i < num_particles; i++)
	{
		weights[i] = particles[i].weight;
		if(particles[i].weight > max_weight)
		{
			max_weight = particles[i].weight;
		}
	}
	
	// debug
	//std::cout << "max weight: " << max_weight << std::endl;

	// new particles vector
	vector<Particle> newParticles;
	newParticles.resize(num_particles);

	// create a uniform real distribution
	std::uniform_real_distribution<double> dist_beta_adder(0., 2*max_weight);

	// implementation of Resampling Wheel
	for(int i = 0; i < num_particles; i++)
	{
		beta += dist_beta_adder(gen);

		// debug
		//std::cout << "beta =" << beta << std::endl;		
		//std::cout << "index =" << index << std::endl;		

		while(weights[index] < beta)
		{
			// debug
			//std::cout << "weights[index] < beta <=> " << weights[index] << " < " << beta << std::endl;		

			beta = beta - weights[index];
			index = (index + 1) % (num_particles-1);

			// debug
			//std::cout << "new beta = " << beta << std::endl;		
			//std::cout << "new index = " << index << std::endl;		
		}
		
		// add selected particle to new particle collection
		newParticles[i] = particles[index];
		newParticles[i].id = i;

		//debug
		//std::cout << " add particle with index " << index << std::endl;
		//std::cout << " example new particle: " << newParticles[0].id << " (" << newParticles[0].x << ", " << newParticles[0].y << ", " << newParticles[0].theta << ") " << "w=" << newParticles[0].weight << std::endl;
	}

	// replace the old particles with new one
	particles = newParticles;
	
	// debug
	//printParticles('R');
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	
	// debug
	//std::cout << "- calling SetAssociations" << std::endl;

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
