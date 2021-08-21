/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

namespace {

int getClosestLandmark(
    const std::vector<Map::single_landmark_s>& predicted,
    const LandmarkObs& observation) {
    double minDistance = std::numeric_limits<double>::infinity();
    int closest = -1;
    for (int i = 0; i < predicted.size(); ++i) {
        const auto& landmark = predicted.at(i);
        double distance = dist(
            landmark.x_f,
            landmark.y_f,
            observation.x,
            observation.y);

        if (distance < minDistance) {
            minDistance = distance;
            closest = i;
        }
    }
    return closest;
}

void addNoise(Particle& particle, std::default_random_engine&  gen, double std[]) {
    std::normal_distribution<double> dist_x(0,std[0]);
    std::normal_distribution<double> dist_y(0,std[1]);
    std::normal_distribution<double> dist_theta(0,std[2]);
    particle.x += dist_x(gen);
    particle.y += dist_y(gen);
    particle.theta += dist_theta(gen);
}

void predict(
    Particle& particle,
    double delta_t,
    double velocity,
    double yaw_rate) {

    if (std::abs(yaw_rate) > 1e-6) {
        double vel_rate = velocity/ yaw_rate;
        double new_theta = particle.theta + yaw_rate * delta_t;
        particle.x += vel_rate * (std::sin(new_theta) - std::sin(particle.theta));
        particle.y += vel_rate * (-std::cos(new_theta) + std::cos(particle.theta));
        particle.theta = new_theta;
    } else {
        particle.x += velocity * std::cos(particle.theta) * delta_t;
        particle.y += velocity * std::sin(particle.theta) * delta_t;
    }
}


std::vector<Map::single_landmark_s> getLandmarksInRange(
    const std::vector<Map::single_landmark_s>& landmarks,
    double x,
    double y,
    double range
    ) {
    std::vector<Map::single_landmark_s> result;
    result.reserve(landmarks.size());
    for (const auto& landmark : landmarks) {
        if (dist(landmark.x_f, landmark.y_f, x, y) <= range) {
            result.push_back(landmark);
        }
    }
    return result;
}

LandmarkObs makeGlobal(const LandmarkObs& obs, const Particle& particle) {
    auto global = obs;
    double cos = std::cos(particle.theta);
    double sin = std::sin(particle.theta);
    global.x = obs.x * cos - obs.y * sin + particle.x;
    global.y =  obs.x * sin + obs.y  * cos + particle.y;
    return global;
}

std::vector<Particle> resampling(const std::vector<Particle>& particles) {
    std::vector<double> weights;
    weights.reserve(particles.size());
    double totalWeight = 0;
    for (const auto& particle : particles) {
        totalWeight += particle.weight;
        weights.emplace_back(totalWeight);
    }

    std::vector<Particle> result;
    result.reserve(particles.size());

    std::default_random_engine gen;
    std::uniform_real_distribution<double> weightDist(0.0, totalWeight);

    while (result.size() < particles.size()) {
        int index = std::lower_bound(weights.begin(), weights.end(), weightDist(gen)) - weights.begin();
        result.emplace_back(particles.at(index));
    }
    return result;
}

}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    std::default_random_engine gen;

    num_particles = 10;
    while (particles.size() < num_particles) {
        int id = particles.size();
        particles.emplace_back(Particle{
            id,
            x,
            y,
            theta,
            1,
            {}, // associations
            {}, // sense_x
            {} // sense_y
        });
        addNoise(particles.back(), gen, std);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(
    double delta_t,
    double std_pos[],
    double velocity,
    double yaw_rate) {
    std::default_random_engine gen;

    for (auto& particle : particles) {
        predict(particle, delta_t, velocity, yaw_rate);
        addNoise(particle, gen, std_pos);
    }
}

double getDensity(double dx, double dy, double sigma_x, double sigma_y) {
    dx /= sigma_x;
    dy /= sigma_y;
    double pow = - (dx * dx + dy * dy)/ 2;
    return std::exp(pow) / (2 * M_PI * sigma_x * sigma_y);
}

double ParticleFilter::calculateWeight(
    const Particle& particle,
    double sensor_range,
    double std_landmark[],
    const std::vector<LandmarkObs>& observations,
    const Map& map_landmarks
    ) const  {
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[0];
    const auto predicted = getLandmarksInRange(map_landmarks.landmark_list, particle.x, particle.y, sensor_range);
    double weight = 1;
    if (predicted.empty()) {
        return 1;
    }
    for (const auto& obs : observations) {
        auto global = makeGlobal(obs, particle);
        int landMarkId = getClosestLandmark(predicted, global);
        if (landMarkId >= 0) {
            const auto& landmark = predicted.at(landMarkId);
            weight *= getDensity(
                global.x - landmark.x_f,
                global.y - landmark.y_f,
                sigma_x,
                sigma_y
                );
        }
    }
    return weight;
}

void ParticleFilter::updateWeights(
    double sensor_range,
    double std_landmark[],
    const std::vector<LandmarkObs>& observations,
    const Map& map_landmarks
) {
    for (auto& particle : particles) {
        particle.weight = calculateWeight(particle, sensor_range, std_landmark, observations, map_landmarks);
    }
}

void ParticleFilter::resample() {
    particles = resampling(particles);
}

void ParticleFilter::SetAssociations(
    Particle& particle,
    const std::vector<int>& associations,
    const std::vector<double>& sense_x,
    const std::vector<double>& sense_y
) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}