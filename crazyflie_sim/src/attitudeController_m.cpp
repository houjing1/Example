//
// Created by houjing1 on 06/08/18.
//

#include "crazyflie_sim/attitudeController_m.h"
#include "crazyflie_sim/utility.h"
#include "crazyflie_sim/state.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>

// constant pi value
const double pi = boost::math::constants::pi<double>();

// Static data member
Parameter AttitudeController_m::param;

// Constructor
AttitudeController_m::AttitudeController_m(const int& id_value):BaseController(), id(id_value), dt(param.mellinger_cycle * 1.0e-6){

}

/*
 *
 * Perform update on the entire controller and return output vector as desired motor input variation.
 * This function overrides its base class function.
 *
 * inputs: setpoint is a Setpoint obejct that contains desired euler angle(rad), angular velocity(rad/s) [desired_rpy(rad), desired_pqr(rad/s)]
 *         state is a State object that contains the measured angular velocity(rad/s) and current rotation matrix(Rwb) [measured_pqr(rad/s), current_R]
 * outputs: base class Control object that contains desired motor input variation(pwm): [desired_motor_var]
 */
const Control& AttitudeController_m::update_overall(const Setpoint& setpoint, const State& state){

    // Get input variables
    const Eigen::Vector3d& desired_euler = setpoint.euler;              // get desired Euler angle, expect radian
    const Eigen::Vector3d& desired_pqr   = setpoint.angular_velocity;   // get desired angular velocity, expect rad/s
    const Eigen::Vector3d& measured_pqr  = state.get_omega();           // get measured angular velocity, expect rad/s
    const Eigen::Matrix3d& current_R     = state.get_R();               // get current rotation matrix, expect Rwb

    // Compute desired R from desired Euler angle set
    const Eigen::Matrix3d& desired_R = get_R_from_euler(desired_euler);

    // [eR]

    /*
    // Get desired axis from desired R
    const Eigen::Vector3d& x_axis_desired = desired_R.col(0);
    const Eigen::Vector3d& y_axis_desired = desired_R.col(1);
    const Eigen::Vector3d& z_axis_desired = desired_R.col(2);

    // Compute quaternion from current R
    const Eigen::Vector4d& q = get_quaternion_from_R(current_R);

    // Fast version (generated using Mathematica)
    const double& x = q(0);
    const double& y = q(1);
    const double& z = q(2);
    const double& w = q(3);

    Eigen::Vector3d eR2;
    //eR2(0) = (-1 + 2*std::sqrt(x) + 2*std::sqrt(y))*y_axis_desired(2) + z_axis_desired(1) - 2*(x*y_axis_desired(0)*z + y*y_axis_desired(1)*z - x*y*z_axis_desired(0) + std::sqrt(x)*z_axis_desired(1) + std::sqrt(z)*z_axis_desired(1) - y*z*z_axis_desired(2)) + 2*w*(-(y*y_axis_desired(0)) - z*z_axis_desired(0) + x*(y_axis_desired(1) + z_axis_desired(2)));
    eR2(0) = (-1 + 2*sqrt(x) + 2*sqrt(y))*y_axis_desired(2) + z_axis_desired(1) - 2*(x*y_axis_desired(0)*z + y*y_axis_desired(1)*z - x*y*z_axis_desired(0) + sqrt(x)*z_axis_desired(1) + sqrt(z)*z_axis_desired(1) - y*z*z_axis_desired(2)) + 2*w*(-(y*y_axis_desired(0)) - z*z_axis_desired(0) + x*(y_axis_desired(1) + z_axis_desired(2)));
    eR2(1) = x_axis_desired(2) - z_axis_desired(0) - 2*(std::sqrt(x)*x_axis_desired(2) + y*(x_axis_desired(2)*y - x_axis_desired(1)*z) - (std::sqrt(y) + std::sqrt(z))*z_axis_desired(0) + x*(-(x_axis_desired(0)*z) + y*z_axis_desired(1) + z*z_axis_desired(2)) + w*(x*x_axis_desired(1) + z*z_axis_desired(1) - y*(x_axis_desired(0) + z_axis_desired(2))));
    eR2(2) = y_axis_desired(0) - 2*(y*(x*x_axis_desired(0) + y*y_axis_desired(0) - x*y_axis_desired(1)) + w*(x*x_axis_desired(2) + y*y_axis_desired(2))) + 2*(-(x_axis_desired(2)*y) + w*(x_axis_desired(0) + y_axis_desired(1)) + x*y_axis_desired(2))*z - 2*y_axis_desired(0)*std::sqrt(z) + x_axis_desired(1)*(-1 + 2*std::sqrt(x) + 2*std::sqrt(z));
    */

    Eigen::Matrix3d eRM = (desired_R.transpose()*current_R) - (current_R.transpose() * desired_R);

    // vee mapping
    Eigen::Vector3d eR;
    eR(0) = eRM(2,1);
    eR(1) = eRM(0,2);
    eR(2) = eRM(1,0);

    // [ew]
    double err_d_roll = 0.0;
    double err_d_pitch = 0.0;

    Eigen::Vector3d ew;

    ew(0) =  desired_pqr(0) - measured_pqr(0);
    ew(1) =  desired_pqr(1) - measured_pqr(1);
    ew(2) =  desired_pqr(2) - measured_pqr(2);

    if(!flag){
        flag = true;
    }
    else{
        err_d_roll = ((desired_pqr(0) - prev_setpoint_omega_roll) - (measured_pqr(0) - prev_omega_roll)) / dt;
        err_d_pitch = ((desired_pqr(1) - prev_setpoint_omega_pitch) - (measured_pqr(1) - prev_omega_pitch)) / dt;
    }

    prev_omega_roll = measured_pqr(0);
    prev_omega_pitch = measured_pqr(1);
    prev_setpoint_omega_roll = desired_pqr(0);
    prev_setpoint_omega_pitch = desired_pqr(1);

    // Integral Error
    i_error_m_x += (-eR(0)) * dt;
    i_error_m_x = clamp(i_error_m_x, -i_range_m_xy, i_range_m_xy);

    i_error_m_y += (-eR(1)) * dt;
    i_error_m_y = clamp(i_error_m_y, -i_range_m_xy, i_range_m_xy);

    i_error_m_z += (-eR(2)) * dt;
    i_error_m_z = clamp(i_error_m_z, -i_range_m_z, i_range_m_z);

    // Moment:
    Eigen::Vector3d M;

    M(0) = -kR_xy * eR(0) + kw_xy * ew(0) + ki_m_xy * i_error_m_x + kd_omega_rp * err_d_roll;
    M(1) = -kR_xy * eR(1) + kw_xy * ew(1) + ki_m_xy * i_error_m_y + kd_omega_rp * err_d_pitch;
    M(2) = -kR_z  * eR(2) + kw_z  * ew(2) + ki_m_z  * i_error_m_z;

    //std::cout<< kd_omega_rp * err_d_roll << " " << kd_omega_rp * err_d_pitch <<std::endl;
    //std::cout<<"err_d: "<< err_d_roll << " " << err_d_pitch <<std::endl;

    // Output
    motor_var_desired(0) = clamp(M(0), -32000, 32000);
    motor_var_desired(1) = clamp(M(1), -32000, 32000);
    motor_var_desired(2) = clamp(M(2), -32000, 32000);

    output.motor_variation = motor_var_desired;

    return output;
}


/*
 * Reset controller internal states.
 * This function overrides its base class function.
 */
void AttitudeController_m::reset_controller(){
    i_error_m_x = 0;
    i_error_m_y = 0;
    i_error_m_z = 0;
}