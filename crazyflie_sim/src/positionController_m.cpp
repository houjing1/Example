//
// Created by houjing1 on 06/08/18.
//

#include "crazyflie_sim/positionController_m.h"
#include "crazyflie_sim/utility.h"
#include "crazyflie_sim/state.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>

// constant pi value
const double pi = boost::math::constants::pi<double>();

// Static data member
Parameter PositionController_m::param;

// Constructor

PositionController_m::PositionController_m(const int& id_value):BaseController(), id(id_value), dt(param.mellinger_cycle * 1.0e-6){

}

/*
 * Perform update on the entire controller and return output vector as desired thrust-roll-pitch.
 * This function overrides its base class function.
 *
 * inputs: setpoint is a Setpoint object that contains the desired position, velocity, acceleration and yaw angle(rad) [desired_pos, desired_vel, desired_acc, desired_yaw]
 *         state is a State object that contains the measured position, velocity, and current rotation matrix [measured_pos, measured_vel, current_R]
 * outputs: base class Control object that contains desired thrust(pwm) and desired euler angle(rad): [desired_thrust desired_euler]
 */
const Control& PositionController_m::update_overall(const Setpoint& setpoint, const State& state){

    const Eigen::Vector3d& desired_pos  = setpoint.position;        // get desired position
    const Eigen::Vector3d& measured_pos = state.get_position();     // get measured position
    const Eigen::Vector3d& desired_vel  = setpoint.velocity;        // get desired velocity
    const Eigen::Vector3d& measured_vel = state.get_velocity();     // get measured velocity
    const Eigen::Vector3d& desired_acc  = setpoint.acceleration;    // get desired acceleration
    const Eigen::Matrix3d& current_R    = state.get_R();            // get current rotation matrix, expect Rwb
    const double &desired_yaw = setpoint.euler(2);                  // get desired yaw angle, expect radian


    // Position and Velocity Error (ep & ev)
    Eigen::Vector3d r_error = desired_pos - measured_pos; // difference between desired and measured position
    Eigen::Vector3d v_error = desired_vel - measured_vel; // difference between desired and measured velocity

    // Integral error
    i_error_z += r_error(2) * dt;
    i_error_z = clamp(i_error_z, -i_range_z, i_range_z);

    i_error_x += r_error(0) * dt;
    i_error_x = clamp(i_error_x, -i_range_xy, i_range_xy);

    i_error_y += r_error(1) * dt;
    i_error_y = clamp(i_error_y, -i_range_xy, i_range_xy);

    // Desired thrust [F_des]
    Eigen::Vector3d target_thrust;

    target_thrust(0) = param.m * desired_acc(0)             + kp_xy * r_error(0) + kd_xy * v_error(0) + ki_xy * i_error_x;
    target_thrust(1) = param.m * desired_acc(1)             + kp_xy * r_error(1) + kd_xy * v_error(1) + ki_xy * i_error_y;
    target_thrust(2) = param.m * (desired_acc(2) + param.g) + kp_z  * r_error(2) + kd_z  * v_error(2) + ki_z  * i_error_z;

    // Z-Axis [zB]
    Eigen::Vector3d z_axis = current_R.col(2);

    // Current thrust [F]
    double current_thrust = target_thrust.dot(z_axis);

    // Calculate axis [zB_des]
    Eigen::Vector3d z_axis_desired = target_thrust.normalized();

    // [xC_des]
    Eigen::Vector3d x_c_des(std::cos(desired_yaw),
                            std::sin(desired_yaw),
                            0.0);

    // [yB_des]
    Eigen::Vector3d y_axis_desired = z_axis_desired.cross(x_c_des).normalized();

    // [xB_des]
    Eigen::Vector3d x_axis_desired = y_axis_desired.cross(z_axis_desired);

    // [R_des]
    R_desired.col(0) = x_axis_desired;
    R_desired.col(1) = y_axis_desired;
    R_desired.col(2) = z_axis_desired;

    // Output
    euler_desired = get_euler_from_R(R_desired);

    /*
     * Mapping Equation
     * T_des = m*a_des = 4*Ct*w^2 and w = a1*PWM + a2
     */

    thrust_desired = (1/param.a1) * (std::sqrt(current_thrust / (param.Ct * 4)) - param.a2); // convert force to pwm, expect total acc
    //thrust_desired = c_2 * current_thrust * current_thrust + c_1 * current_thrust + c; // convert force to pwm, expect total acc

    output.thrust = thrust_desired;
    output.euler = euler_desired;

    return output;

}

/*
 * brief reset controller
 *
 * Reset controller internal states.
 * This function overrides its base class function.
 */
void PositionController_m::reset_controller(){
    i_error_x = 0;
    i_error_y = 0;
    i_error_z = 0;
}
