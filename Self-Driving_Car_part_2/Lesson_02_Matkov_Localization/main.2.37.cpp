#include <algorithm>
#include <iostream>
#include <vector>

#include "helpers.h"

// function to get pseudo ranges
std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions,
                                     float pseudo_position);

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(std::vector<float> landmark_positions,
                        std::vector<float> observations, std::vector<float> pseudo_ranges,
                        float distance_max, float observation_stdev);

int main() {  
    // set observation standard deviation:
    float observation_stdev = 1.0f;

    // number of x positions on map
    int map_size = 25;

    // set distance max
    float distance_max = map_size;

    // define landmarks
    std::vector<float> landmark_positions {5, 10, 12, 20};

    // define observations
    std::vector<float> observations {5.5, 13, 15};

    // step through each pseudo position x (i)
    for (int i = 0; i < map_size; ++i) {
        float pseudo_position = float(i);

        // get pseudo ranges
        std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions,
                                                             pseudo_position);

        //get observation probability
        float observation_prob = observation_model(landmark_positions, observations,
                                                   pseudo_ranges, distance_max,
                                                   observation_stdev);
        //print to stdout
        std::cout << observation_prob << std::endl;
    }

    return 0;
}

// TODO: Complete the observation model function
// calculates likelihood prob term based on landmark proximity
float observation_model(
    std::vector<float> landmark_positions,
    std::vector<float> observations,
    std::vector<float> pseudo_ranges,
    float distance_max,
    float observation_stdev
    ) {
    float distance_prob = 1;
    // YOUR CODE HERE
    for (size_t i = 0; i < observations.size(); ++i) {
        float range = i < pseudo_ranges.size() ?
            range = pseudo_ranges.at(i) :
            std::numeric_limits<float>::infinity();
        distance_prob *= Helpers::normpdf(observations[i], range, observation_stdev);
    }
    return distance_prob;
}

std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions,
                                     float pseudo_position) {
    // define pseudo observation vector
    std::vector<float> pseudo_ranges;
            
    // loop over number of landmarks and estimate pseudo ranges
    for (int l=0; l< landmark_positions.size(); ++l) {
        // estimate pseudo range for each single landmark
        // and the current state position pose_i:
        float range_l = landmark_positions[l] - pseudo_position;

        // check if distances are positive:
        if (range_l > 0.0f) {
            pseudo_ranges.push_back(range_l);
        }
    }

    // sort pseudo range vector
    sort(pseudo_ranges.begin(), pseudo_ranges.end());

    return pseudo_ranges;
}