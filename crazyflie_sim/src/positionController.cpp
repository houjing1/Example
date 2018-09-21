/*
 * positionController.cpp
 *
 *  Created On : 27/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/positionController.h"
#include "crazyflie_sim/utility.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>

// constant pi value
const double pi = boost::math::constants::pi<double>();

// Static data member
Parameter PositionController::param;

// Constructor
PositionController::PositionController(const int& id_value, const DroneSimInfo& info):
        BaseController(),
        id(id_value),
        x_PID(id_value, "x_position"),
        y_PID(id_value, "y_position"),
        z_PID(id_value, "z_position"),
        vx_PID(id_value, "x_velocity"),
        vy_PID(id_value, "y_velocity"),
        vz_PID(id_value, "z_velocity")
{
    set_up_controller(id_value, info);
}

// set up all Pid objects
void PositionController::set_up_controller(const int& id_value, const DroneSimInfo& info){

    //setup kp, ki, kd and integration limit
    x_PID.set_kp(info.get_x_pid(0))
            .set_ki(info.get_x_pid(1))
            .set_kd(info.get_x_pid(2))
            .set_integral_limit(info.get_x_pid(3));

    y_PID.set_kp(info.get_y_pid(0))
            .set_ki(info.get_y_pid(1))
            .set_kd(info.get_y_pid(2))
            .set_integral_limit(info.get_y_pid(3));

    z_PID.set_kp(info.get_z_pid(0))
            .set_ki(info.get_z_pid(1))
            .set_kd(info.get_z_pid(2))
            .set_integral_limit(info.get_z_pid(3));

    vx_PID.set_kp(info.get_vx_pid(0))
            .set_ki(info.get_vx_pid(1))
            .set_kd(info.get_vx_pid(2))
            .set_integral_limit(info.get_vx_pid(3));

    vy_PID.set_kp(info.get_vy_pid(0))
            .set_ki(info.get_vy_pid(1))
            .set_kd(info.get_vy_pid(2))
            .set_integral_limit(info.get_vy_pid(3));

    vz_PID.set_kp(info.get_vz_pid(0))
            .set_ki(info.get_vz_pid(1))
            .set_kd(info.get_vz_pid(2))
            .set_integral_limit(info.get_vz_pid(3));

    //setup output limit
    x_PID.set_output_limit(param.xyVelMax * param.velMaxOverhead);
    y_PID.set_output_limit(param.xyVelMax * param.velMaxOverhead);
    z_PID.set_output_limit(std::max(0.5,param.zVelMax * param.velMaxOverhead));

    vx_PID.set_output_limit(param.rpLimit * param.rpLimitOverhead);
    vy_PID.set_output_limit(param.rpLimit * param.rpLimitOverhead);
    vz_PID.set_output_limit(65536.0 / param.thrustScale / 2.0);

    //setup frequency
    x_PID.set_dt(param.position_loop_cycle * 1.0e-6);
    y_PID.set_dt(param.position_loop_cycle * 1.0e-6);
    z_PID.set_dt(param.position_loop_cycle * 1.0e-6);
    vx_PID.set_dt(param.position_loop_cycle * 1.0e-6);
    vy_PID.set_dt(param.position_loop_cycle * 1.0e-6);
    vz_PID.set_dt(param.position_loop_cycle * 1.0e-6);
}

// perform update on position sub controller
// inputs: desired position vector, measured position vector
// output: vel_desired vector
const Eigen::Vector3d& PositionController::update_position_pid(const Eigen::Vector3d& pos_desired, const Eigen::Vector3d& pos_measured){

    // set desired values for x,y,z pid
    x_PID.set_desired(pos_desired(0));
    y_PID.set_desired(pos_desired(1));
    z_PID.set_desired(pos_desired(2));

    // update x,y,z pid controllers to obtain velocity set points
    vel_desired(0) = x_PID.update_pid(pos_measured(0), true);
    vel_desired(1) = y_PID.update_pid(pos_measured(1), true);
    vel_desired(2) = z_PID.update_pid(pos_measured(2), true);

    return vel_desired;
}

// perform update on velocity sub controller
// inputs: desired velocity vector, measured velocity vector, measured yaw angle(rad)
// output: trp_desired vector
const Eigen::Vector3d& PositionController::update_velocity_pid(const Eigen::Vector3d& vel_desired, const Eigen::Vector3d& vel_measured, const double& yaw_measured){

    // set desired values for vx,vy,vz pid
    vx_PID.set_desired(vel_desired(0));
    vy_PID.set_desired(vel_desired(1));
    vz_PID.set_desired(vel_desired(2));

    // update x,y,z pid controllers to obtain intermediate acc values
    acc_desired(0) = vx_PID.update_pid(vel_measured(0), true);
    acc_desired(1) = vy_PID.update_pid(vel_measured(1), true);
    acc_desired(2) = vz_PID.update_pid(vel_measured(2), true);

    // calculate and bound desired thrust

    //std::cout << "controller: " << acc_desired(2)  << std::endl;

    /*
     * Linear Mapping Equation
     */

    double thrust_desired = acc_desired(2) * param.thrustScale + param.thrustBase; // convert m/s/s to pwm, expect relative acc

    /*
     * Non-linear Mapping Equation
     * T_des = m*a_des = 4*Ct*w^2 and w = a1*PWM + a2
     */

    //double thrust_desired = (1/param.a1) * (std::sqrt(param.m*acc_desired(2) / (param.Ct * 4)) - param.a2); // convert m/s/s to pwm, expect total acc
    //double thrust_desired = (-73820.0*0.033*0.033) * acc_desired(2)  * acc_desired(2)  + (147723.96*0.033) * acc_desired(2)  + (1369.68); // convert m/s/s to pwm, expect total acc

    if (thrust_desired < param.thrustMin){
        thrust_desired = param.thrustMin;
    }

    // calculate and bound desired roll and pitch
    double roll_desired = -(acc_desired(1) * std::cos(yaw_measured)) + (acc_desired(0) * std::sin(yaw_measured));
    double pitch_desired = (acc_desired(0) * std::cos(yaw_measured)) + (acc_desired(1) * std::sin(yaw_measured));

    roll_desired = clamp(roll_desired, -param.rpLimit, param.rpLimit);
    pitch_desired = clamp(pitch_desired, -param.rpLimit, param.rpLimit);

    // update trp_desired attribute
    trp_desired(0) = thrust_desired;
    trp_desired(1) = roll_desired;
    trp_desired(2) = pitch_desired;

    // update output Control object

    output.thrust = trp_desired(0);     // add thrust
    output.euler(0) = trp_desired(1);   // add roll(deg)
    output.euler(1) = trp_desired(2);   // add pitch(deg)

    return trp_desired;
}

// override the Base class function
// perform overall update on position controller
// input: Setpoint and State object for position controller that contains [pos_desired, pos_measured, vel_measured, yaw_measured]
// output: base class Control object that contains desired thrust, roll, pitch
const Control& PositionController::update_overall(const Setpoint& setpoint, const State& state){

    // combine update_position and update_velocity to produce overall update
    update_position_pid(setpoint.position, state.get_position());
    update_velocity_pid(vel_desired, state.get_velocity(), state.get_euler()(2)); // expect radian

    return output;
}

// override the Base class function
// reset all Pid objects' internal states
void PositionController::reset_controller(){

    x_PID.reset_pid();
    y_PID.reset_pid();
    z_PID.reset_pid();

    vx_PID.reset_pid();
    vy_PID.reset_pid();
    vz_PID.reset_pid();
}
