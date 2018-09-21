/*
 * droneSimIfo.cpp
 *
 *  Created On : 25/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/droneSimInfo.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

// Check for out of range index
void DroneSimInfo::check_index( const int& index, const int& size ) const{

    if (index > (size - 1)){
        std::cerr << "[DroneSimInfo]: index(" << index << ") out of range!"  << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Overloaded check_index
void DroneSimInfo::check_index( const int& index) const{

    if (index > (num_of_drone - 1)){
        std::cerr << "[DroneSimInfo]: index(" << index << ") out of range!"  << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Modify container size
void DroneSimInfo::resize(const int& size){

    num_of_drone = size;
    name.resize(size);
    x_init.conservativeResize(size);
    y_init.conservativeResize(size);
}

