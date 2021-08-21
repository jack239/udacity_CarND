#include <iostream>
#include "help_functions.h"

float parameter = 1.0; // set as control parameter or observation measurement
float stdev = 1.0; // position or observation standard deviation

int main() {

    std::cout << Helpers::normpdf(5.5, 5, stdev) << std::endl;
    std::cout << Helpers::normpdf(11, 11, stdev) << std::endl;
    std::cout << Helpers::normpdf(11, 11, stdev) * Helpers::normpdf(5.5, 5, stdev) << std::endl;

    return 0;
}