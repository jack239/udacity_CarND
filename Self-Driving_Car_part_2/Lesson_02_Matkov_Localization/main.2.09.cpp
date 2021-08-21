#include <iostream>
#include <vector>
#include <cmath>

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
std::vector<float> initialize_priors(
    int map_size,
    std::vector<float> landmark_positions,
    float position_stdev);

int main() {
    // set standard deviation of position
    float position_stdev = 1.0f;

    // set map horizon distance in meters
    int map_size = 25;

    // initialize landmarks
    std::vector<float> landmark_positions {5, 10, 20};

    // initialize priors
    std::vector<float> priors = initialize_priors(
        map_size,
        landmark_positions,
        position_stdev);

//     print values to stdout
    for (int p = 0; p < priors.size(); ++p) {
        std::cout << priors[p] << std::endl;
    }

    return 0;
}

// TODO: Complete the initialize_priors function
std::vector<float> initialize_priors(
    int map_size,
    std::vector<float> landmark_positions,
    float position_stdev) {

    // initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev

    // set all priors to 0.0
    std::vector<float> priors(map_size, 0.0);
    float sum = 0;
    for (float lendmark : landmark_positions) {
        int start = std::floor(lendmark - position_stdev);
        start = std::max(start, 0);
        int end = std::ceil(lendmark + position_stdev);
        end = std::min<int>(end, map_size - 1);
//        std::cout << lendmark << " " << start << " " << end << std::endl;
        for (size_t i = start; i <= end; ++i) {
            priors[i] += 1;
            sum += 1;
        }
    }
    if (sum > 0) {
        for (float& pr : priors) {
            pr /= sum;
        }
    }

    return priors;
}