/*
 * attitudeController.cpp
 *
 *  Created On : 27/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/attitudeController.h"
#include "crazyflie_sim/utility.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>

// constant pi value
const double pi = boost::math::constants::pi<double>();

// Static data memeber
Parameter AttitudeController::param;

// Constructor
AttitudeController::AttitudeController(const int &id_value, const DroneSimInfo &info):
        BaseController(),
        id(id_value),
        roll_PID(id_value, "roll"),
        pitch_PID(id_value, "pitch"),
        yaw_PID(id_value, "yaw"),
        p_PID(id_value, "x_angular_velocity"),
        q_PID(id_value, "y_angular_velocity"),
        r_PID(id_value, "z_angular_velocity")
{
    set_up_controller(id_value, info);
}

// set up all Pid objects
void AttitudeController::set_up_controller(const int& id_value, const DroneSimInfo& info){

    //setup kp, ki, kd and integration limit
    roll_PID.set_kp(info.get_roll_pid(0))
            .set_ki(info.get_roll_pid(1))
            .set_kd(info.get_roll_pid(2))
            .set_integral_limit(info.get_roll_pid(3));

    pitch_PID.set_kp(info.get_pitch_pid(0))
             .set_ki(info.get_pitch_pid(1))
             .set_kd(info.get_pitch_pid(2))
             .set_integral_limit(info.get_pitch_pid(3));

    yaw_PID.set_kp(info.get_yaw_pid(0))
            .set_ki(info.get_yaw_pid(1))
            .set_kd(info.get_yaw_pid(2))
            .set_integral_limit(info.get_yaw_pid(3));

    p_PID.set_kp(info.get_p_pid(0))
            .set_ki(info.get_p_pid(1))
            .set_kd(info.get_p_pid(2))
            .set_integral_limit(info.get_p_pid(3));

    q_PID.set_kp(info.get_q_pid(0))
            .set_ki(info.get_q_pid(1))
            .set_kd(info.get_q_pid(2))
            .set_integral_limit(info.get_q_pid(3));

    r_PID.set_kp(info.get_r_pid(0))
            .set_ki(info.get_r_pid(1))
            .set_kd(info.get_r_pid(2))
            .set_integral_limit(info.get_r_pid(3));

    //setup output limit

    /**default limit is implemented**/

    //setup frequency
    roll_PID.set_dt(param.outer_loop_cycle * 1.0e-6);
    pitch_PID.set_dt(param.outer_loop_cycle  * 1.0e-6);
    yaw_PID.set_dt(param.outer_loop_cycle  * 1.0e-6);
    p_PID.set_dt(param.inner_loop_cycle * 1.0e-6);
    q_PID.set_dt(param.inner_loop_cycle * 1.0e-6);
    r_PID.set_dt(param.inner_loop_cycle * 1.0e-6);

}

// perform update on angle sub controller
// inputs: desired roll pitch yaw, measured roll pitch yaw
// output: pqr_desired vector
const Eigen::Vector3d& AttitudeController::update_angle_pid(const Eigen::Vector3d& rpy_desired, const Eigen::Vector3d& rpy_measured){

    // set desired values for roll, pitch pid
    roll_PID.set_desired(rpy_desired(0));
    pitch_PID.set_desired(rpy_desired(1));

    // set and bound error for yaw pid
    double yaw_error = rpy_desired(2) - rpy_measured(2);
    if(yaw_error > 180.0){
        yaw_error -= 360.0;
    }
    else if (yaw_error < -180.0){
        yaw_error += 360.0;
    }
    yaw_PID.set_error(yaw_error);

    // update roll, pitch, yaw pid controllers to obtain angular rate set points
    pqr_desired(0) = roll_PID.update_pid(rpy_measured(0), true);
    pqr_desired(1) = pitch_PID.update_pid(rpy_measured(1), true);
    pqr_desired(2) = yaw_PID.update_pid(rpy_measured(2), false);

    return pqr_desired;
}

// perform update on angular rate sub controller
// inputs: desired angular velocity, measured angular velocity
// output motor_var_desired vector
const Eigen::Vector3d& AttitudeController::update_angular_rate_pid(const Eigen::Vector3d& pqr_desired, const Eigen::Vector3d& pqr_measured){

    // set desired values for p,q,r pid
    p_PID.set_desired(pqr_desired(0));
    q_PID.set_desired(pqr_desired(1));
    r_PID.set_desired(pqr_desired(2));

    // update p, q, r pid controllers to obtain motor input variations
    motor_var_desired(0) = p_PID.update_pid(pqr_measured(0), true);
    motor_var_desired(1) = q_PID.update_pid(pqr_measured(1), true);
    motor_var_desired(2) = r_PID.update_pid(pqr_measured(2), true);

    // update output Control object
    output.motor_variation = motor_var_desired; // add motor input variation

    return motor_var_desired;
}

// override the Base class function
// perform overall update
// input: Setpoint object to provide rpy_desired(deg) and State object to provide rpy_measured(rad) and pqr_measured(rad)
// output: base class Control object that contains desired motor variation(pwm)
const Control& AttitudeController::update_overall(const Setpoint& setpoint, const State& state){

    // combine update_angle and update_angular_rate to produce overall update
    update_angle_pid(setpoint.euler, state.get_euler() * 180.0 / pi);     // expect degree
    update_angular_rate_pid(pqr_desired, state.get_omega() * 180.0 / pi); // expect degree

    return output;
}

// override the Base class function
// reset all Pid objects' internal states
void AttitudeController::reset_controller(){

    roll_PID.reset_pid();
    pitch_PID.reset_pid();
    yaw_PID.reset_pid();

    p_PID.reset_pid();
    q_PID.reset_pid();
    r_PID.reset_pid();
}
