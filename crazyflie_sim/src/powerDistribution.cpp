/*
 * powerDistribution.cpp
 *
 *  Created On : 28/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/powerDistribution.h"
#include "crazyflie_sim/utility.h"

// Static data member
Parameter PowerDistribution::param;

// Compute 4 motor output pwm from motor input variation and base thrust
// Input: motor_variation as a 3 by 1 [x,y,z] vector and base_thrust as a double
// Output: motor_pwm attribute
const Eigen::Vector4d& PowerDistribution::compute_motor_pwm(const Eigen::Vector3d& motor_variation, const double& base_thrust){

    motor_pwm(0) = clamp( base_thrust + (-motor_variation(0) - motor_variation(1)) / 2 - motor_variation(2),
            0.0, thrust_limit);
    motor_pwm(1) = clamp( base_thrust + (-motor_variation(0) + motor_variation(1)) / 2 + motor_variation(2),
            0.0, thrust_limit);
    motor_pwm(2) = clamp( base_thrust + ( motor_variation(0) + motor_variation(1)) / 2 - motor_variation(2),
            0.0, thrust_limit);
    motor_pwm(3) = clamp( base_thrust + ( motor_variation(0) - motor_variation(1)) / 2 + motor_variation(2),
            0.0, thrust_limit);

    // remove fluctuations in motor pwm
    average_data(data_matrix, motor_pwm);

    return motor_pwm;
}
