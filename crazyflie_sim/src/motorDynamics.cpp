/*
 * motorDynamics.cpp
 *
 *  Created On : 28/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */
#include "crazyflie_sim/motorDynamics.h"

// Static data memeber
Parameter MotorDynamics::param;

// Compute 4 motor output rpm from 4 motor ouput pwm
// Input: motor_pwm as 4 by 1 [motor1,motor2,motor3,motor4] vector
// Output: motor_rpm attribute
const Eigen::Vector4d& MotorDynamics::compute_motor_rpm(const Eigen::Vector4d& motor_pwm){

    motor_rpm(0) = param.a1 * motor_pwm(0) + param.a2;
    motor_rpm(1) = param.a1 * motor_pwm(1) + param.a2;
    motor_rpm(2) = param.a1 * motor_pwm(2) + param.a2;
    motor_rpm(3) = param.a1 * motor_pwm(3) + param.a2;

    return motor_rpm;

}
