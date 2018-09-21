/*
 * utility.cpp
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/utility.h"
#include <cmath>

// generate number from random normal distribution
double normal_distribution_generator(double mean, double std) {

    // construct a trivial random generator engine from a time-based seed:
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    std::normal_distribution<double> distribution (mean, std);

    double value =  distribution(generator);

    return value;
}


