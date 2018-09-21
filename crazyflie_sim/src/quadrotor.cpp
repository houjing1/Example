/*
 * quadrotor.cpp
 *
 *  Created On : 29/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include <boost/math/constants/constants.hpp>
#include <random>
#include <iostream>
#include <vector>
#include "crazyflie_sim/quadrotor.h"
#include "crazyflie_sim/utility.h"
#include "crazyflie_sim/external_force.h"


// constant pi value
const double pi = boost::math::constants::pi<double>();

// Static data member
Parameter Quadrotor::param;

// constructor
// pass true for att_only only under the situation of receiving "AldHold" commands only
// the default value for att_only is false if not passing any parameter
Quadrotor::Quadrotor(const int& id_value, const DroneSimInfo& info, std::string type, std::string reference, bool use_multi_markers, bool disturbance_force, bool measurement_noise, bool att_only):
        id(id_value),
        controller_type(type),
        reference_source(reference),
        multi_markers_flag(use_multi_markers),
        attitude_only(att_only),
        disturbance_flag(disturbance_force),
        noise_flag(measurement_noise),
        true_state(id_value, info),
        estimated_state(id_value, info),
        position_controller(id_value, info),
        attitude_controller(id_value, info),
        position_controller_m(id_value),
        attitude_controller_m(id_value),
        power_distribution(id_value),
        motor_dynamics(id_value),
        quadrotor_dynamics(id_value),
        latest_cmd(id_value)
{
    if(id_value < 1){
        std::cerr << "[Quadrotor " << id <<"]: invalid id number " << id_value << " (id starts from 1)"  << std::endl;
        exit(1);
    }

    // construct a vector of pointer to external force functions
    if (disturbance_flag){
        //external_force.push_back(random_force); // add in random force
        external_force.push_back(random_walk); // add in random walk force
    }

}

// Utility functions

// update position controller to update command_euler and thrust_actuator set points for attitude controller
// no need for calling this function if attitude only mode is active
void Quadrotor::update_position_controller(){

    // skip this function call if only attitude controller is intended to be used
    if(attitude_only){
        return;
    }

    DroneCmd command = get_latest_cmd();        // get latest DroneCmd for this drone

    // skip update if drone is near ground
    if(near_ground(command)){
        shutdown();
        return;
    }

    // extract PosSet command
    if(command.get_mode() == "PosSet"){         // command has mode "PosSet"

        // set up arguments and return variables for controller
        Setpoint setpoint;      // used to setup desired value to position controller
        Control output;         // used to capture output value from update_overall
        State current_state;    // used to provide measured value to position controller

        // get desired position from command
        setpoint.position << command.get_cmd(0), command.get_cmd(1), command.get_cmd(2); // get desired position(x,y,z vector)

        // get measured states from true state
        if(reference_source == "true_state"){
            current_state = get_state();
        }

        // get measured states from estimator
        else if(reference_source == "estimated_state"){
            current_state = get_estimated_state();

            // get rotation info from true state if single marker is used
            if(!multi_markers_flag){

                {
                    std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
                    current_state.set_R(true_state.get_R());
                    current_state.set_euler(true_state.get_euler());
                    current_state.set_omega(true_state.get_omega());
                }

                // SECTION TO ADD NOISE //
                if(noise_flag){
                    // add noise to measurements
                }
            }
        }

        else{
            std::cerr<< "[Quadrotor " << id << "]: Unknown reference source type: " << reference_source  << std::endl;
            exit(1);
        }

        if(controller_type == "pid"){

            // update pid position controller
            {
                std::unique_lock<std::shared_timed_mutex> lock(position_controller_mutex);
                output = position_controller.update_overall(setpoint, current_state);
            }

            // update new set points in thrust_actuator
            {
                std::unique_lock<std::shared_timed_mutex> lock(thrust_mutex);
                thrust_actuator = output.thrust;            // thrust(pwm)
            }

            // update new set points in command_euler member
            {
                std::unique_lock<std::shared_timed_mutex> lock(euler_mutex);
                command_euler(0) = output.euler(0) * (pi / 180.0);                       // roll angle(rad), convert deg to rad
                command_euler(1) = output.euler(1) * (pi / 180.0);                       // pitch angle(rad), convert deg to rad
                command_euler(2) = command.get_cmd(3);                                   // yaw angle(rad)
            }
        }

        else if(controller_type == "Mellinger"){

            // desired velocity, acceleration are zero

            setpoint.euler(2) = command.get_cmd(3); // add desired yaw(rad)

            // update Mellinger's position controller
            {
                std::unique_lock<std::shared_timed_mutex> lock(position_controller_mutex);
                output = position_controller_m.update_overall(setpoint, current_state);
            }

            // update new set points in thrust_actuator
            {
                std::unique_lock<std::shared_timed_mutex> lock(thrust_mutex);
                thrust_actuator = output.thrust;            // thrust(pwm)
            }

            // update new set points in command_euler member
            {
                std::unique_lock<std::shared_timed_mutex> lock(euler_mutex);
                command_euler = output.euler; // expect radian
            }
        }

        else{
            std::cerr<< "[Quadrotor " << id << "]: Unknown controller type: " << controller_type << std::endl;
            exit(1);
        }
    }

    else if (command.get_mode() == "AltHold"){
        // do nothing for command that has mode "AltHold"
    }

    else{   // command has incorrect mode
        std::cerr<< "[Quadrotor " << id << "]: Trying to update position controller with unknown command: " << command.get_mode() << std::endl;
        exit(1);
    }
}

// update angle sub controller of attitude controller
void Quadrotor::update_attitude_controller_outer(){

    //transition_reset();                         // check command transition and reset controller states

    // get the command for attitude controller
    DroneCmd command = get_latest_cmd();        // get latest DroneCmd for this drone

    // extract AltHold command
    if (command.get_mode() == "AltHold"){

        // get and update new set points in thrust_actuator directly from AltHold command
        {
            std::unique_lock<std::shared_timed_mutex> lock(thrust_mutex);
            thrust_actuator = command.get_cmd(2);            // thrust(pwm)
        }
        // get and update new set points in command_euler directly from AlHold command
        {
            std::unique_lock<std::shared_timed_mutex> lock(euler_mutex);
            command_euler(0) = command.get_cmd(0);           // roll angle(rad)
            command_euler(1) = command.get_cmd(1);           // pitch angle(rad)
            command_euler(2) = command.get_cmd(3);           // yaw angle(rad)
        }
    }

        // case for incorrect commands
    else if (attitude_only || command.get_mode() != "PosSet"){
        std::cerr<< "[Quadrotor " << id << "]: Trying to update attitude controller with command: " << command.get_mode() << std::endl;
        exit(1);
    }

    Eigen::Vector3d rpy_desired;
    Eigen::Vector3d rpy_measured;

    // get desired roll, pitch, yaw from command_euler member
    {
        std::shared_lock<std::shared_timed_mutex> lock(euler_mutex);
        rpy_desired = command_euler * (180.0 / pi);                          // roll, pitch, yaw vector(deg), convert from rad to deg
    }

    if(reference_source == "true_state"){
        // get measured roll, pitch, yaw set from true state
        {
            std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
            rpy_measured = true_state.get_euler() * (180.0 / pi);            // roll, pitch, yaw vector(deg), conversion is needed
        }
    }

    else if(reference_source == "estimated_state"){
        // get measured euler from estimator if multi-markers are used
        if(multi_markers_flag){
            {
                std::shared_lock<std::shared_timed_mutex> lock(estimate_mutex);
                rpy_measured = estimated_state.get_euler() * (180.0 / pi);
            }
        }

        // get measured euler from true state if single marker is used
        else{
            {
                std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
                rpy_measured = true_state.get_euler() * (180.0 / pi);
            }
            // SECTION TO ADD NOISE //
            if(noise_flag){
                // add noise to measurements
            }
        }
    }

    else{
        std::cerr<< "[Quadrotor " << id << "]: Unknown reference source type: " << reference_source  << std::endl;
        exit(1);
    }

    // update angle sub-controller from attitude controller
    {
        std::unique_lock<std::shared_timed_mutex> lock(attitude_controller_mutex);
        attitude_controller.update_angle_pid(rpy_desired, rpy_measured);
    }
}

// update angular rate sub controller of attitude controller
void Quadrotor::update_attitude_controller_inner(){

    Eigen::Vector3d pqr_measured;
    Eigen::Vector3d output;

    if(reference_source == "true_state"){
        // get measured angular velocity set from true state
        {
            std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
            pqr_measured = true_state.get_omega() * (180.0 / pi);                 // x, y, z components of angular velocity, conversion is needed
        }
    }

    else if(reference_source == "estimated_state"){
        // get measured euler from estimator if multi-markers are used
        if(multi_markers_flag){
            {
                std::shared_lock<std::shared_timed_mutex> lock(estimate_mutex);
                pqr_measured = estimated_state.get_omega() * (180.0 / pi);
            }
        }

        // get measured euler from true state if single marker is used
        else{
            {
                std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
                pqr_measured = true_state.get_omega() * (180.0 / pi);
            }
            // SECTION TO ADD NOISE //
            if(noise_flag){
                // add noise to measurements
            }
        }
    }

    else{
        std::cerr<< "[Quadrotor " << id << "]: Unknown reference source type: " << reference_source  << std::endl;
        exit(1);
    }

    // update angular rate sub-controller
    {
        std::unique_lock<std::shared_timed_mutex> lock(attitude_controller_mutex);
        output = attitude_controller.update_angular_rate_pid(attitude_controller.get_pqr_desired(),pqr_measured);
    }

    // update motor input variation
    {
        std::unique_lock<std::shared_timed_mutex> lock(motor_var_mutex);
        motor_variation = output;
    }
}

// update attitude controller to update motor_variation member
// this function could be called independently from update_position_controllers
void Quadrotor::update_attitude_controller(){

    //transition_reset();                         // check command transition and reset controller states

    // get the command for attitude controller
    DroneCmd command = get_latest_cmd();        // get latest DroneCmd for this drone

    // skip update if drone is near ground
    if(near_ground(command)){
        shutdown();
        return;
    }

    // extract AltHold command
    if (command.get_mode() == "AltHold"){

        // get and update new set points in thrust_actuator directly from AltHold command
        {
            std::unique_lock<std::shared_timed_mutex> lock(thrust_mutex);
            thrust_actuator = command.get_cmd(2);            // thrust(pwm)
        }
        // get and update new set points in command_euler directly from AlHold command
        {
            std::unique_lock<std::shared_timed_mutex> lock(euler_mutex);
            command_euler(0) = command.get_cmd(0);           // roll angle(rad)
            command_euler(1) = command.get_cmd(1);           // pitch angle(rad)
            command_euler(2) = command.get_cmd(3);           // yaw angle(rad)
        }
    }

    // case for incorrect commands
    else if (attitude_only || command.get_mode() != "PosSet"){
        std::cerr<< "[Quadrotor " << id << "]: Trying to update attitude controller with command: " << command.get_mode()  << std::endl;
        exit(1);
    }

    // set up arguments and return variables for controller
    Setpoint setpoint;      // used to setup desired value to position controller
    Control output;         // used to capture output value from update_overall
    State current_state;    // used to provide measured value to position controller

    // get measured states from true state
    if(reference_source == "true_state"){
        current_state = get_state();
    }

    // get measured states from estimator
    else if(reference_source == "estimated_state"){

        // get measured rotation from estimator if multi-markers are used
        if(multi_markers_flag){
            current_state = get_estimated_state();
        }

        // get measured rotation from true state if single marker is used
        else{
            current_state = get_state();

            // SECTION TO ADD NOISE //
            if(noise_flag){
                // add noise to measurements
            }
        }
    }

    else{
        std::cerr<< "[Quadrotor " << id << "]: Unknown reference source type: " << reference_source  << std::endl;
        exit(1);
    }

    if(controller_type == "pid"){
        // get desired roll, pitch, yaw from command_euler member
        {
            std::shared_lock<std::shared_timed_mutex> lock(euler_mutex);
            setpoint.euler = command_euler * (180.0 / pi); // get desired roll, pitch, yaw vector(deg), convert from rad to deg
        }

        // update attitude controller
        {
            std::unique_lock<std::shared_timed_mutex> lock(attitude_controller_mutex);
            output = attitude_controller.update_overall(setpoint, current_state);
        }
    }

    else if(controller_type == "Mellinger"){
        // desired angular velocity is zero

        // get desired roll, pitch, yaw from command_euler member
        {
            std::shared_lock<std::shared_timed_mutex> lock(euler_mutex);
            setpoint.euler = command_euler ; // get desired roll, pitch, yaw vector(rad)
        }

        // update attitude controller
        {
            std::unique_lock<std::shared_timed_mutex> lock(attitude_controller_mutex);
            output = attitude_controller_m.update_overall(setpoint, current_state);
        }
    }

    else{
        std::cerr<< "[Quadrotor " << id << "]: Unknown " << controller_type << " controller type" << std::endl;
        exit(1);
    }

    // update motor input variation
    {
        std::unique_lock<std::shared_timed_mutex> lock(motor_var_mutex);
        motor_variation = output.motor_variation;
    }
}

// update state member by performing power distribution, motor dynamics and drone dynamics calculations
// input: vector of pointer to external force functions
void Quadrotor::update_motor_output(){

    Eigen::Vector3d motor_var_desired;
    Eigen::Vector4d motor_pwm_desired;
    Eigen::Vector4d motor_rpm_desired;
    double base_thrust;
    State current_state;

    // get base thrust from thrust_actuator member
    {
        std::shared_lock<std::shared_timed_mutex> lock(thrust_mutex);
        base_thrust = thrust_actuator;
    }
    // get desired motor variation from motor_variation member
    {
        std::shared_lock<std::shared_timed_mutex> lock(motor_var_mutex);
        motor_var_desired = motor_variation;

    }

    // compute desired motor pwm and rpm
    motor_pwm_desired = power_distribution.compute_motor_pwm(motor_var_desired, base_thrust);
    motor_rpm_desired = motor_dynamics.compute_motor_rpm(motor_pwm_desired);

    // update motor_pwm member
    {
        std::unique_lock<std::shared_timed_mutex> lock(motor_pwm_mutex);
        motor_pwm = motor_pwm_desired;
    }
    // update motor_rpm member
    {
        std::unique_lock<std::shared_timed_mutex> lock(motor_rpm_mutex);
        motor_rpm = motor_rpm_desired;
    }

    // get current state member
    {
        std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
        current_state = true_state;
    }

    // update current state object using drone dynamics
    // either update_state, update_state2, update_state3 could be used to compute drone dynamics
    quadrotor_dynamics.update_state3(current_state, motor_rpm_desired, external_force);

    // update state member
    {
        std::unique_lock<std::shared_timed_mutex> lock(state_mutex);
        true_state = current_state;
    }

    //std::cout<< "----------------------"<<std::endl;
    //std::cout<< current_state.get_id()<<std::endl;
    //std::cout<< "pos: "<<current_state.get_position()(0) << " " << current_state.get_position()(1) << " " << current_state.get_position()(2) <<std::endl;
    //std::cout<< "vel: "<<current_state.get_velocity()(0) << " " << current_state.get_velocity()(1) << " " << current_state.get_velocity()(2) <<std::endl;
    //std::cout<< "acc: "<<current_state.get_acceleration()(0) << " " << current_state.get_acceleration()(1) << " " << current_state.get_acceleration()(2) <<std::endl;
    //std::cout<< "euler: "<<current_state.get_euler()(0) << " " << current_state.get_euler()(1) << " " << current_state.get_euler()(2) <<std::endl;

}

// reset position or attitude controllers
// input: a string that is either "position" or "attitude" to indicate which controller to reset
void Quadrotor::reset_controller(const std::string& type){

    if(type == "position"){

        std::unique_lock<std::shared_timed_mutex> lock(position_controller_mutex);
        if(controller_type == "pid"){
            position_controller.reset_controller();
        }
        else if(controller_type == "Mellinger"){
            position_controller_m.reset_controller();
        }
    }
    else if(type == "attitude"){

        std::unique_lock<std::shared_timed_mutex> lock(attitude_controller_mutex);
        if(controller_type == "pid"){
            attitude_controller.reset_controller();
        }
        else if(controller_type == "Mellinger"){
            attitude_controller_m.reset_controller();
        }
    }
    else{
        std::cerr<< "[Quadrotor " << id << "]: Trying to reset controller with unknown type: " << type << std::endl;
        exit(1);
    }
}

// check if command mode has changed and reset corresponding controller states
void Quadrotor::transition_reset(){

    // get current and previous command mode
    std::string current_mode = get_latest_cmd().get_mode(); // thread safe
    std::string previous_mode = get_previous_cmd_mode(); // thread safe

    if(current_mode != previous_mode){  // transition occurs

        // set previous mode to current mode
        set_previous_cmd_mode(current_mode); // thread safe

        if(previous_mode != "NA" && current_mode == "AltHold"){ // transition from "PosSet" to "AltHold"
            reset_controller("position");
            std::cout<< "[Quadrotor " << id << "]: reset position controller"<< std::endl;
        }


        else if(previous_mode != "NA" && current_mode == "PosSet"){ // transition from "AltHold" to "PosSet"
            reset_controller("attitude");
            std::cout<< "[Quadrotor " << id << "]: reset attitude controller"<< std::endl;
        }

    }
}

// shutdown drone by turning of motor and reseting commands and controllers
void Quadrotor::shutdown(){

    reset_controller("position");
    reset_controller("attitude");

    set_thrust_actuator(0.0);
    set_command_euler(Eigen::Vector3d::Zero());
    set_motor_variation(Eigen::Vector3d::Zero());
    set_motor_pwm(Eigen::Vector4d::Zero());
    set_motor_rpm(Eigen::Vector4d::Zero());

}

// check if drone is commanded to be near ground
// input: reference to the latest cmd object
bool Quadrotor::near_ground(const DroneCmd& cmd){
    return(cmd.get_mode() == "PosSet" && cmd.get_cmd(2) < 0.001);
}

// generate position vector from state member
// output: 3 by 1 vector of latest drone position in world frame
Eigen::Vector3d Quadrotor::generate_position(){

    std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
    Eigen::Vector3d pos = true_state.get_position();

    // add vicon noise if required
    if(noise_flag){
        pos += multivariate_normal<double, 3>(param.vicon_mean, param.vicon_cov);
    }

    return pos;
}
// overloaded generate_position function
// input: reference to a 3 by 1 vector
// output: update reference with the latest drone position in world frame
void Quadrotor::generate_position(Eigen::Vector3d& pos){

    std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
    pos = true_state.get_position();

    // add vicon noise if required
    if(noise_flag){
        pos += multivariate_normal<double, 3>(param.vicon_mean, param.vicon_cov);
    }
}

// generate list of position vectors from state member to indicate marker positions
// input: reference to a vector of 3 by 1 vector of latest marker positions
// output: update the reference with marker positions in world frame
// Pw = Twb Pb where Twb = [Rwb, Pbw_w; 0, 1]
void Quadrotor::generate_markers_position(std::vector<Eigen::Vector3d>& pos_list){

    Eigen::Vector3d pos;
    Eigen::Matrix3d R;

    // get copy of current position and current R_wb
    {
        std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
        pos = true_state.get_position();
        R = true_state.get_R();
    }

    // generate position of markers in world frame
    pos_list[0] = R * param.positive_x + pos;
    pos_list[1] = R * param.negative_x + pos;
    pos_list[2] = R * param.positive_y + pos;

    // add vicon noise if required
    if(noise_flag){
        pos_list[0] += multivariate_normal<double, 3>(param.vicon_mean, param.vicon_cov);
        pos_list[1] += multivariate_normal<double, 3>(param.vicon_mean, param.vicon_cov);
        pos_list[2] += multivariate_normal<double, 3>(param.vicon_mean, param.vicon_cov);
    }
}


// Accessor functions

// get true state
State Quadrotor::get_state(){
    std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
    State current_state(true_state);
    return current_state;
}

void Quadrotor::get_state(State& current_state){
    std::shared_lock<std::shared_timed_mutex> lock(state_mutex);
    current_state = true_state;
}

// get estimated state
State Quadrotor::get_estimated_state(){
    std::shared_lock<std::shared_timed_mutex> lock(estimate_mutex);
    State state_object(estimated_state);
    return state_object;
}

void Quadrotor::get_estiamted_state(State& state_object){
    std::shared_lock<std::shared_timed_mutex> lock(estimate_mutex);
    state_object = estimated_state;
}

// get previous_cmd_mode
std::string Quadrotor::get_previous_cmd_mode(){
    std::shared_lock<std::shared_timed_mutex> lock(mode_mutex);
    std::string mode = previous_cmd_mode;
    return mode;
}

void Quadrotor::get_previous_cmd_mode( std::string& mode){
    std::shared_lock<std::shared_timed_mutex> lock(mode_mutex);
    mode = previous_cmd_mode;
}

// get latest_cmd
DroneCmd Quadrotor::get_latest_cmd(){
    std::shared_lock<std::shared_timed_mutex> lock(cmd_mutex);
    DroneCmd cmd = latest_cmd;
    return cmd;
}

void Quadrotor::get_latest_cmd(DroneCmd& cmd){
    std::shared_lock<std::shared_timed_mutex> lock(cmd_mutex);
    cmd = latest_cmd;
}

// get thrust_actuator
double Quadrotor::get_thrust_actuator(){
    std::shared_lock<std::shared_timed_mutex> lock(thrust_mutex);
    double thrust = thrust_actuator;
    return thrust;
}

void Quadrotor::get_thrust_actuator(double& thrust){
    std::shared_lock<std::shared_timed_mutex> lock(thrust_mutex);
    thrust = thrust_actuator;
}

// get command_euler
Eigen::Vector3d Quadrotor::get_command_euler(){
    std::shared_lock<std::shared_timed_mutex> lock(euler_mutex);
    Eigen::Vector3d euler = command_euler;
    return euler;
}

void Quadrotor::get_command_euler(Eigen::Vector3d& euler){
    std::shared_lock<std::shared_timed_mutex> lock(euler_mutex);
    euler = command_euler;
}

// get motor_variation
Eigen::Vector3d Quadrotor::get_motor_variation(){
    std::shared_lock<std::shared_timed_mutex> lock(motor_var_mutex);
    Eigen::Vector3d variation = motor_variation;
    return variation;
}

void Quadrotor::get_motor_variation(Eigen::Vector3d& variation){
    std::shared_lock<std::shared_timed_mutex> lock(motor_var_mutex);
    variation = motor_variation;
}

// get motor_pwm
Eigen::Vector4d Quadrotor::get_motor_pwm(){
    std::shared_lock<std::shared_timed_mutex> lock(motor_pwm_mutex);
    Eigen::Vector4d pwm = motor_pwm;
    return pwm;
}

void Quadrotor::get_motor_pwm(Eigen::Vector4d& pwm){
    std::shared_lock<std::shared_timed_mutex> lock(motor_pwm_mutex);
    pwm = motor_pwm;
}

// get motor_rpm
Eigen::Vector4d Quadrotor::get_motor_rpm(){
    std::shared_lock<std::shared_timed_mutex> lock(motor_rpm_mutex);
    Eigen::Vector4d rpm = motor_rpm;
    return rpm;
}

void Quadrotor::get_motor_rpm(Eigen::Vector4d& rpm){
    std::shared_lock<std::shared_timed_mutex> lock(motor_rpm_mutex);
    rpm = motor_rpm;
}

// get controller type

std::string Quadrotor::get_controller_type(){
    return controller_type;
}
void Quadrotor::get_controller_type(std::string& type){
    type = controller_type;
}


// Mutator functions

void Quadrotor::set_latest_cmd(const DroneCmd& cmd){
    std::unique_lock<std::shared_timed_mutex> lock(cmd_mutex);
    latest_cmd = cmd;
}

void Quadrotor::set_previous_cmd_mode(const std::string& mode){
    std::unique_lock<std::shared_timed_mutex> lock(mode_mutex);
    previous_cmd_mode = mode;
}

void Quadrotor::set_controller_type(const std::string& type){
    controller_type = type;
}

void Quadrotor::set_estimated_state(const State& state_object){
    std::unique_lock<std::shared_timed_mutex> lock(estimate_mutex);
    estimated_state = state_object;
}

void Quadrotor::set_thrust_actuator(const double& thrust_value){
    std::unique_lock<std::shared_timed_mutex> lock(thrust_mutex);
    thrust_actuator = thrust_value;
}

void Quadrotor::set_command_euler(const Eigen::Vector3d& euler_value){
    std::unique_lock<std::shared_timed_mutex> lock(euler_mutex);
    command_euler = euler_value;
}

void Quadrotor::set_motor_variation(const Eigen::Vector3d& var_value){
    std::unique_lock<std::shared_timed_mutex> lock(motor_var_mutex);
    motor_variation = var_value;
}

void Quadrotor::set_motor_pwm(const Eigen::Vector4d& pwm_value){
    std::unique_lock<std::shared_timed_mutex> lock(motor_pwm_mutex);
    motor_pwm = pwm_value;
}

void Quadrotor::set_motor_rpm(const Eigen::Vector4d& rpm_value){
    std::unique_lock<std::shared_timed_mutex> lock(motor_rpm_mutex);
    motor_rpm = rpm_value;
}


