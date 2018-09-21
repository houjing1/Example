/*
 * droneSim.cpp
 *
 *  Created On : 06/07/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "crazyflie_sim/droneSim.h"
#include "crazyflie_sim/utility.h"
#include <boost/lexical_cast.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <thread>

#include <algorithm>
#include <random>
#include <chrono>

// Constructor that takes a drone number integer and ROS node handle as arguments
DroneSim::DroneSim(const int& drone_num, ros::NodeHandle& handle):
        num_of_drone(drone_num),
        info(drone_num),
        nh(handle),
        nh_slow(ros::NodeHandle()),
        nh_fast(ros::NodeHandle()),
        spinner_general(1, &queue_general),
        spinner_slow(1, &queue_slow),
        spinner_fast(1, &queue_fast)
{

    ROS_INFO_STREAM("[Simulator]: Initializing simulator...");

    // set up custom callback queues for each node handle
    nh.setCallbackQueue(&queue_general);
    nh_slow.setCallbackQueue(&queue_slow);
    nh_fast.setCallbackQueue(&queue_fast);

    // update DroneSimInfo attribute and check for success
    if(!setup_DroneSimInfo()){
        ROS_FATAL_STREAM("[Simulator]: DroneSimInfo failed to update");
        exit(1);
    }
    else{ROS_INFO_STREAM("[Simulator]: DroneSimInfo updated");}

    // call initialization functions
    initialize_gazebo_msg();                // initialize the vectors of StateModel objects
    initialize_vicon_msg();                 // initialize the vicon_msg object
    initialize_drone_list();                // initialize the vector of Quadrotor objects
    initialize_subscribers();               // initialize subscriber
    initialize_publishers();                // initialize publisher
    initialize_timers();                    // initialize timers

    // get other flags from ROS server
    ros::param::param<bool>("simulator/detect_collision", detect_collision, false);
    ROS_INFO_STREAM("[Simulator]: Collision detection: " << (detect_collision ? "on" : "off"));

    ROS_INFO_STREAM("[Simulator]: Initialization complete");
}

// function to initialize vectors of ModelState objects
void DroneSim::initialize_gazebo_msg(){

    // get use_gazebo flag from ROS server
    ros::param::param<bool>("simulator/use_gazebo", use_gazebo, false);
    if(use_gazebo){ // if use_gazebo is true
        ROS_INFO_STREAM("[Simulator]: Gazebo simulator used");
        ROS_INFO_STREAM("[Simulator]: Initializing Vectors of StateModel");

        for(int i = 0; i < num_of_drone; i++){
            gazebo_msg_sim.push_back(gazebo_msgs::ModelState());
            gazebo_msg_sim[i].model_name = info.get_name(i);

            gazebo_msg_des.push_back(gazebo_msgs::ModelState());
            gazebo_msg_des[i].model_name = info.get_name(i) + "_tag";
        }
    }
}

// function to initialize a Markers object as publisher message to Estimator
void DroneSim::initialize_vicon_msg(){

    int number = num_of_drone;

    // get use_multi_markers flag from ROS server
    ros::param::param<bool>("simulator/use_multi_markers", use_multi_markers, false);
    if(use_multi_markers){
        ROS_INFO_STREAM("[Simulator]: Initializing vicon_msg using multiple markers");
        pos_list = std::vector<Eigen::Vector3d> (3, Eigen::Vector3d::Zero());           // initialize the vector of marker positions
        number *= 3;                                                                    // total number of Marker object in vicon message
    }
    else{ ROS_INFO_STREAM("[Simulator]: Initializing vicon_msg using single marker"); }

    // create a dynamic Markers object as message to estimator
    vicon_msg_ptr = dsl__utilities__msg::MarkersPtr(new dsl__utilities__msg::Markers);
    for(int i = 0; i < number; i++){
        vicon_msg_ptr -> markers.push_back(dsl__utilities__msg::Marker());;
    }

    // get random_shuffle flag from ROS server
    ros::param::param<bool>("simulator/random_shuffle", random_shuffle, false);
    ROS_INFO_STREAM("[Simulator]: Random shuffle of vicon markers is " << (random_shuffle ? "on" : "off"));

}

// function to initialize a vector of Quadrotor objects
void DroneSim::initialize_drone_list(){
    // get info form DroneSimInfo
    std::string controller_type = info.get_controller_type();
    std::string reference_source = info.get_reference_source();
    bool noise_flag = info.get_noise_flag();
    bool disturbance_flag = info.get_disturbance_flag();

    ROS_INFO_STREAM("[Simulator]: Initializing Vector of Quadrotor base on " << controller_type << " controller");
    ROS_INFO_STREAM("[Simulator]: Initializing Vector of Quadrotor using " << reference_source << " as controller reference");
    ROS_INFO_STREAM("[Simulator]: Initializing Vector of Quadrotor with measurement noise: " << (noise_flag? "on" : "off"));
    ROS_INFO_STREAM("[Simulator]: Initializing Vector of Quadrotor with disturbance force: " << (disturbance_flag? "on" : "off"));

    for(int i = 0; i < num_of_drone; i++){
        drones.emplace_back(new Quadrotor(i+1, info, controller_type, reference_source, use_multi_markers, disturbance_flag, noise_flag));
    }
}

// Initialization function for subscribers
// using subscriber_callback function
void DroneSim::initialize_subscribers(){
    ROS_INFO_STREAM("[Simulator]: Initializing Subscribers");

    // get command subscriber topic name from ROS server
    std::string cmd_topic;
    if(ros::param::param<std::string>("simulator/cmd_topic", cmd_topic, "/SwarmCmds")){
        ROS_INFO_STREAM("[Simulator]: " << cmd_topic << " is used as command subscriber topic");
    }
    else{ ROS_INFO_STREAM("[Simulator]: No subscriber topic found under name cmd_topic, default command subscriber topic used"); }

    subscriber_cmd = nh.subscribe(cmd_topic, 30, &DroneSim::cmd_subscriber_callback, this);

    // get full-state estimate subscriber topic name from ROS server
    std::string state_topic;
    if(ros::param::param<std::string>("simulator/state_topic", state_topic, "/full_state")){
        ROS_INFO_STREAM("[Simulator]: " << state_topic << " is used as full-state estimate subscriber topic");
    }
    else{ ROS_INFO_STREAM("[Simulator]: No subscriber topic found under name state_topic, default full-state estimate subscriber topic used"); }

    subscriber_state = nh.subscribe(state_topic, 30, &DroneSim::state_subscriber_callback, this);
}

// Initialization function for publishers
void DroneSim::initialize_publishers(){
    ROS_INFO_STREAM("[Simulator]: Initializing Publishers");

    // gazebo publisher
    if(use_gazebo){
        publisher_gazebo = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 30);
    }

    // get vicon marker topic from ROS server
    std::string pub_topic;
    if(ros::param::param<std::string>("simulator/pub_topic", pub_topic, "/vicon/markers")){
        ROS_INFO_STREAM("[Simulator]: " << pub_topic << " is used as publisher topic");
    }  // get publish rate from ROS server
    else{ ROS_INFO_STREAM("[Simulator]: No publisher topic found under name pub_topic, default publisher topic used"); }

    // vicon publisher
    publisher_marker = nh.advertise<dsl__utilities__msg::Markers>(pub_topic, 30);

    // test publisher
    test_publisher = nh.advertise<std_msgs::Bool>("/test", 1);                                 /**test freq publisher**/

    // drone's full state publisher
    _pub_swarmstates = nh.advertise<crazyflie_estimator::SwarmStates>("sim_state", 30);
}

// Initialization function for timers
void DroneSim::initialize_timers(){

    // set up timers for PID controller
    if(info.get_controller_type() == "pid"){
        ROS_INFO_STREAM("[Simulator]: Initializing timers base on " << info.get_controller_type() << " controller");

        // control loop timers
        position_loop_timer = nh_slow.createTimer(ros::Duration(param.position_loop_cycle * 1.0e-6), &DroneSim::position_loop_timer_callback, this);
        attitude_loop_timer = nh_fast.createTimer(ros::Duration(param.inner_loop_cycle * 1.0e-6), &DroneSim::attitude_loop_timer_callback, this);
        //outer_attitude_loop_timer = nh_fast.createTimer(ros::Duration(param.outer_loop_cycle * 1.0e-6), &DroneSim::outer_attitude_loop_timer_callback, this);
        //inner_attitude_loop_timer = nh_fast.createTimer(ros::Duration(param.inner_loop_cycle * 1.0e-6), &DroneSim::inner_attitude_loop_timer_callback, this);
    }

    // set up timers for Mellinger's controller
    else if(info.get_controller_type() == "Mellinger"){
        ROS_INFO_STREAM("[Simulator]: Initializing timers base on " << info.get_controller_type() << " controller");

        // control loop timers
        attitude_loop_timer = nh_fast.createTimer(ros::Duration(param.mellinger_cycle * 1.0e-6), &DroneSim::combined_loop_timer_callback, this);
    }

    else{
        ROS_FATAL_STREAM("[Simulator]: Initializing timers using controller type that is not implemented");
        exit(1);
    }

    // gazebo timer
    if(use_gazebo){
        gazebo_timer = nh.createTimer(ros::Duration(1.0 / 60.0), &DroneSim::gazebo_timer_callback, this);
    }

    // publisher timers
    double pub_rate;
    ros::param::param<double>("simulator/vicon_pub_rate", pub_rate, 100.0);  // get publish rate from ROS server
    ROS_INFO_STREAM("[Simulator]: vicon marker publish rate: " << pub_rate);
    publish_marker_timer = nh.createTimer(ros::Duration(1.0/pub_rate), &DroneSim::publish_marker_timer_callback, this);
}

// outer loop timer callback function to update position controller
void DroneSim::position_loop_timer_callback(const ros::TimerEvent&){

    // perform update on position controller
    for(int i = 0; i < num_of_drone; i++){
        if(drones[i]->get_latest_cmd().get_cmd_exist()){
            if(counter > 0){ counter--; return; }  // skip first couple updates to avoid instability
            drones[i]->update_position_controller();
        }
    }
}

// attitude loop timer callback function to update attitude controller and drone dynamics
void DroneSim::attitude_loop_timer_callback(const ros::TimerEvent&){
    // perform update on attitude controller
    for(int i = 0; i < num_of_drone; i++){
        if(drones[i]->get_latest_cmd().get_cmd_exist()){
            drones[i]->update_attitude_controller();
            drones[i]->update_motor_output();
        }
    }
    /** test frequency **/
    //test_publisher.publish(1);
}

// combined position and attitude loop timer callback function to update position, attitude controller and drone dynamics all together
void DroneSim::combined_loop_timer_callback(const ros::TimerEvent&){
    // perform update on attitude controller
    for(int i = 0; i < num_of_drone; i++){
        if(drones[i]->get_latest_cmd().get_cmd_exist()){
            drones[i]->update_position_controller();
            drones[i]->update_attitude_controller();
            drones[i]->update_motor_output();
        }
    }
    /** test frequency **/
    //test_publisher.publish(1);
}

// inner loop timer callback function to update attitude controller and drone dynamics
void DroneSim::outer_attitude_loop_timer_callback(const ros::TimerEvent&){
    // perform update on outer attitude controller
    for(int i = 0; i < num_of_drone; i++){
        if(drones[i]->get_latest_cmd().get_cmd_exist()){
            drones[i]->update_attitude_controller_outer();
        }
    }
}

// callback function for inner_attitude_loop_timer
void DroneSim::inner_attitude_loop_timer_callback(const ros::TimerEvent&){
    // perform update on inner attitude controller and drone dynamics
    for(int i = 0; i < num_of_drone; i++){
        if(drones[i]->get_latest_cmd().get_cmd_exist()){
            drones[i]->update_attitude_controller_inner();
            drones[i]->update_motor_output();
        }
    }
}

// callback function for publish_marker_timer
void DroneSim::publish_marker_timer_callback(const ros::TimerEvent &){

    if(!use_multi_markers){ // case of single marker

        Eigen::Vector3d pos;
        // add positions to vicon_msg
        for(int i = 0; i < num_of_drone; i++){

            //Eigen::Vector3d pos = drones[i]->generate_position();
            drones[i] -> generate_position(pos);
            vicon_msg_ptr->markers[i].translation.x = pos(0) * 1000.0;
            vicon_msg_ptr->markers[i].translation.y = pos(1) * 1000.0;
            vicon_msg_ptr->markers[i].translation.z = pos(2) * 1000.0;
        }
    }

    else{   // case of multiple markers
        // loop through each drone and get vector of marker postions
        for(int i = 0; i < num_of_drone; i++){
            drones[i] -> generate_markers_position(pos_list);
            // loop through each marker position and add position to vicon message
            auto marker_num = pos_list.size();
            for(int j = 0; j < marker_num; j++){
                vicon_msg_ptr->markers[i*marker_num + j].translation.x = pos_list[j](0) * 1000.0;
                vicon_msg_ptr->markers[i*marker_num + j].translation.y = pos_list[j](1) * 1000.0;
                vicon_msg_ptr->markers[i*marker_num + j].translation.z = pos_list[j](2) * 1000.0;
            }
        }
    }
    vicon_msg_ptr->header.stamp = ros::Time::now();          // add time stamp to pose_msg

    // randomly shuffle the markers to simulate real vicon
    if(random_shuffle){
        long seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle (vicon_msg_ptr->markers.begin(), vicon_msg_ptr->markers.end(), std::default_random_engine(seed));
    }

    // publish marker message
    publisher_marker.publish(vicon_msg_ptr);

    // Package and publish the swarm state
    crazyflie_estimator::SwarmStates swarm_msg;
    for (int i = 0; i < num_of_drone; i++)
    {
        State s;
        drones[i]->get_state(s);
        Eigen::Quaterniond q(s.get_R());
        crazyflie_estimator::FullState state;
        for (int j = 0; j < 3; ++j)
        {
            state.pos[j] = s.get_position()[j];
            state.vel[j] = s.get_velocity()[j];
            state.acc[j] = s.get_acceleration()[j];
            state.quat[j] = q.coeffs()[j];
            state.omega[j] = s.get_omega()[j];
        }
        state.quat[3] = q.coeffs()[3];
        swarm_msg.fullstate.push_back(state);

        //std::cout<< "omega: " <<s.get_omega()(0)<<" "<<s.get_omega()(1)<<" "<<s.get_omega()(2)<<std::endl;

    }
    swarm_msg.header.stamp = ros::Time::now();
    _pub_swarmstates.publish(swarm_msg);

    // check collision
    if(detect_collision){
        check_collision();
    }
}

// callback function for gazebo_timer
void DroneSim::gazebo_timer_callback(const ros::TimerEvent&){

    for(int i = 0; i < num_of_drone; i++){
        publisher_gazebo.publish(gazebo_msg_des[i]);    // publish tag message

        // get position
        Eigen::Vector3d pos = drones[i]->generate_position();
        gazebo_msg_sim[i].pose.position.x = pos(0);
        gazebo_msg_sim[i].pose.position.y = pos(1);
        gazebo_msg_sim[i].pose.position.z = pos(2);

        // get quaternion
        Eigen::Vector4d quaternion = drones[i]->get_state().get_quaternion_from_R();
        gazebo_msg_sim[i].pose.orientation.x = quaternion(0);
        gazebo_msg_sim[i].pose.orientation.y = quaternion(1);
        gazebo_msg_sim[i].pose.orientation.z = quaternion(2);
        gazebo_msg_sim[i].pose.orientation.w = quaternion(3);

        publisher_gazebo.publish(gazebo_msg_sim[i]);    // publish state message
    }
}

// Subscriber callback function to update latest full-state estimation message
void DroneSim::state_subscriber_callback(const crazyflie_estimator::SwarmStatesPtr& state_ptr){

    crazyflie_estimator::FullState state;
    State estimated_state;
    Eigen::Vector4d quat;
    Eigen::Matrix3d R;

    for (int i = 0; i < num_of_drone; i++){

        state = state_ptr->fullstate[i];

        for (int j = 0; j < 3; ++j){

            // set estimated pos, vel and acc
            estimated_state.set_position(j, state.pos[j]);
            estimated_state.set_velocity(j, state.vel[j]);
            estimated_state.set_acceleration(j, state.acc[j]);

            // set estimated omega and quaternion
            if(use_multi_markers) {
                estimated_state.set_omega(j, state.omega[j]); // rad/s
                //estimated_state.set_omega(j, drones[i]->get_state().get_omega()(j)); //rad/s
                quat[j] = state.quat[j];
            }
        }

        // set estimated quaternion
        if(use_multi_markers) {
            quat[3] = state.quat[3];
            R = get_R_from_quaternion(quat); // Rwb
            estimated_state.set_R(R);
            estimated_state.set_euler(get_euler_from_R(R)); // Euler angle in rad
        }

        // set drone's estimated state
        drones[i]->set_estimated_state(estimated_state);

        //std::cout<<estimated_state.get_euler()(1)<<std::endl;

    }

    /** test frequency **/
    test_publisher.publish(1);
}

// Subscriber callback function to update latest_cmd vector from command message
// Able to extract commands from command message with unsorted drone order
void DroneSim::cmd_subscriber_callback(const crazyflie_central::SwarmCmdsPtr& cmd_ptr){

    // check if SwarmCmds message have command data
    if (sizeof(cmd_ptr->cmds) == 0){
        ROS_INFO_STREAM("[Simulator]: no command from received command message");
        return;
    }

    int index_cmd = 0;
    for(auto it_cmd = std::begin(cmd_ptr->names); it_cmd != std::end(cmd_ptr->names); ++it_cmd, ++index_cmd){

        // get the iterator that point to name inside info name list
        auto it_name = std::find(info.get_name().begin(), info.get_name().end(), *it_cmd);

        if (it_name == info.get_name().end()) { // name not found from name vector

            ROS_WARN_STREAM("[Simulator]: No name (" << *it_cmd <<") from command message matches with names from name list");
            continue;
        }
        else { // name found inside info name list

            auto index = std::distance(info.get_name().begin(), it_name);        // get the name index number inside info name list
            DroneCmd cmd_data(static_cast<int>(index+1));                        // create a empty DroneCmd object for this drone

            std::string cmd_type = cmd_ptr->cmds[index_cmd].type;                // get corresponding command type
            if(cmd_type == "PosSet" || cmd_type == "AltHold"){
                cmd_data.set_mode(cmd_type);
            }
            else{ // name not found inside info name list

                ROS_WARN_STREAM("[Simulator]: Unknown command type (" << cmd_type << ") for " << *it_cmd);
                continue;
            }

            // extract "AltHold" command
            if(cmd_type == "AltHold"){

                cmd_data.set_cmd(0, clamp(cmd_ptr->cmds[index_cmd].values[0], -euler_angle_max, euler_angle_max));  // set Roll (rad)
                cmd_data.set_cmd(1, clamp(cmd_ptr->cmds[index_cmd].values[1], -euler_angle_max, euler_angle_max));  // set pitch (rad)

                double thrust = cmd_ptr->cmds[index_cmd].values[2];

                if(info.get_controller_type() == "pid"){

                    //std::cout << "command: " << thrust  << std::endl;

                    /*
                     * Linear Mapping Equation
                     */

                    // convert total acc to relative acc
                    Eigen::Matrix3d current_R = drones[index]->get_estimated_state().get_R();
                    thrust = (thrust - param.g) / current_R(2,2);

                    thrust = thrust * param.thrustScale + param.thrustBase; // convert from m/s/s to pwm, acc is relative

                    /*
                     * Non-linear Mapping Equation
                     * T_des = m*a_des = 4*Ct*w^2 and w = a1*PWM + a2
                     */

                    //thrust = (1/param.a1) * (std::sqrt(param.m*thrust / (param.Ct * 4)) - param.a2);  // convert from m/s/s to pwm, expect total acc
                    //thrust = (-73820.0*0.033*0.033) * thrust * thrust + (147723.96*0.033) * thrust + (1369.68); // convert from m/s/s to pwm, expect total acc
                }

                else{
                    thrust =  (1/param.a1) * (std::sqrt(param.m*thrust / (param.Ct * 4)) - param.a2) ; // convert from m/s/s to pwm, expect total acc
                }

                cmd_data.set_cmd(2, thrust);                                   // set thrust (pwm)
            }

            // extract "PosSet" command
            else{

                cmd_data.set_cmd(0, cmd_ptr->cmds[index_cmd].values[0]);       // set x
                cmd_data.set_cmd(1, cmd_ptr->cmds[index_cmd].values[1]);       // set y
                cmd_data.set_cmd(2, cmd_ptr->cmds[index_cmd].values[2]);       // set z

                // reset controllers if z command is < 0
                if(cmd_ptr->cmds[index_cmd].values[2] < 0){

                    drones[index]->reset_controller("position");
                    drones[index]->reset_controller("attitude");

                    ROS_INFO_STREAM("[Simulator]: Reset controllers of " << info.get_name(index) << " due to z command < 0");
                }
            }

            cmd_data.set_cmd(3, cmd_ptr->cmds[index_cmd].values[3]);        // set yaw (rad)
            cmd_data.set_cmd_exist(1);                                      // set command to be existent

            drones[index]->set_latest_cmd(cmd_data);                        // set latest command for corresponding drone
        }
    }
}

// function that populate DroneSimInfo object with parameters from ROS parameter server
bool DroneSim::setup_DroneSimInfo(){

    bool valid = true;

    XmlRpc::XmlRpcValue ROS_param;
    ros::param::get("simulator", ROS_param);

    // update initial positions
    if(!ROS_param.hasMember("init_pos")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: init_pos not found");
    }
    else{
        if(ROS_param["init_pos"].size() < num_of_drone){ // number of drone initial positions is less than number of drones
            valid = false;
            ROS_WARN_STREAM("[Simulator]: init_pos specifies less positions than number of drones");
        }
        else{ // number of drone initial positions is enough for number of drones
            for(int i = 0; i < num_of_drone; ++i){
                info.set_x_init(i, ROS_param["init_pos"][i][0]);
                info.set_y_init(i, ROS_param["init_pos"][i][1]);
            }
            ROS_INFO_STREAM("[Simulator]: DroneSimInfo - init_pos updated");
        }
    }

    // update name
    if(!ROS_param.hasMember("name")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: name not found");
    }
    else {
        std::ostringstream oss;

        for(int i = 0; i < num_of_drone; ++i){
            //oss << i;
            info.set_name(i, std::string(ROS_param["name"]) + boost::lexical_cast<std::string>(i+1));
        }
        ROS_INFO_STREAM("[Simulator]: DroneSimInfo - name updated");
    }

    // update controller type
    if(!ROS_param.hasMember("controller_type")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: controller type not found");
    }

    else{
        info.set_controller_type(ROS_param["controller_type"]);
        ROS_INFO_STREAM("[Simulator]: DroneSimInfo - " << info.get_controller_type() <<" controller type updated");
    }

    // update reference source
    if(!ROS_param.hasMember("reference_source")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: reference source not found");
    }

    else{
        info.set_reference_source(ROS_param["reference_source"]);
        ROS_INFO_STREAM("[Simulator]: DroneSimInfo - " << info.get_reference_source() <<" reference source updated");
    }

    // update noise flag
    if(!ROS_param.hasMember("include_noise")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: noise flag not found");
    }

    else{
        info.set_noise_flag(ROS_param["include_noise"]);
        ROS_INFO_STREAM("[Simulator]: DroneSimInfo - noise flag updated");
    }

    // update disturbance flag
    if(!ROS_param.hasMember("include_disturbance")){
        valid = false;
        ROS_WARN_STREAM("[Simulator]: disturbance flag not found");
    }

    else{
        info.set_disturbance_flag(ROS_param["include_disturbance"]);
        ROS_INFO_STREAM("[Simulator]: DroneSimInfo - disturbance flag updated");
    }


    // Case when pid controller is used
    if(info.get_controller_type() == "pid"){

        // update PIDs
        if(!ROS_param.hasMember("PIDs")){
            valid = false;
            ROS_WARN_STREAM("[Simulator]: PIDs not found");
        }

        else{

            Eigen::Vector4d pid_vector;
            std::string pid_name;

            // Roll Pid
            pid_name = "Roll";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_roll_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // Pitch Pid
            pid_name = "Pitch";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_pitch_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // Yaw Pid
            pid_name = "Yaw";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_yaw_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // RollRate Pid
            pid_name = "RollRate";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_p_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // PitchRate Pid
            pid_name = "PitchRate";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_q_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // YawRate Pid
            pid_name = "YawRate";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_r_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // X Pid
            pid_name = "X";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_x_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // Y Pid
            pid_name = "Y";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_y_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // Z Pid
            pid_name = "Z";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_z_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // VX Pid
            pid_name = "VX";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_vx_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // VY Pid
            pid_name = "VY";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_vy_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            // VZ Pid
            pid_name = "VZ";
            if(ROS_param["PIDs"].hasMember(pid_name)){
                if ( get_pid_param(ROS_param["PIDs"][pid_name], pid_vector) ){
                    info.set_vz_pid(pid_vector);
                }
                else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing parameters from " << pid_name); }
            }
            else{ valid = false; ROS_WARN_STREAM("[Simulator]: Missing " << pid_name); }

            ROS_INFO_STREAM("[Simulator]: DroneSimInfo - pid parameters updated");
        }
    }

    // Case when Mellinger's controller is used
    else if(info.get_controller_type() == "Mellinger"){

    }

    return valid;
}

// helping function for setup_DroneSimInfo
bool DroneSim::get_pid_param(XmlRpc::XmlRpcValue& pid_param ,Eigen::Vector4d& pid_vector){

    bool valid = true;
    std::vector<std::string> field_name{"kp", "ki", "kd", "iLimit"};

    for(std::vector<std::string>::iterator it = field_name.begin(); it != field_name.end(); ++it){
        if(!pid_param.hasMember(*it)){
            valid = false;
        }
    }

    if(valid){
        pid_vector(0) = static_cast<double>(pid_param["kp"]);
        pid_vector(1) = static_cast<double>(pid_param["ki"]);
        pid_vector(2) = static_cast<double>(pid_param["kd"]);
        pid_vector(3) = static_cast<double>(pid_param["iLimit"]);
    }

    return valid;
}

// function that checks if any pair of drone has collided
bool DroneSim::check_collision(){

    Eigen::Vector3d pos1;
    Eigen::Vector3d pos2;

    for(int i = 0; i < num_of_drone; i++){
        pos1 = drones[i]->get_state().get_position();

        for(int j = i+1; j < num_of_drone; j++){
            pos2 = drones[j]->get_state().get_position();

            if( (param.collision_matrix * (pos1-pos2)).squaredNorm() <= 2.0 ){
                ROS_WARN_STREAM("[Simulator]: Collision has occurred between drone " << i << " and drone " << j);
            }
        }
    }
}


// function that runs ROS node
void DroneSim::run(){

    // spin all threads
    spinner_general.start();
    spinner_slow.start();
    spinner_fast.start();
    ros::waitForShutdown();
    //ros::spin();
}


// nodelet class

class DroneSimNodelet: public nodelet::Nodelet{
public:
    DroneSimNodelet() {};
    ~DroneSimNodelet() {main.join(); delete drone_sim_ptr;}

private:
    virtual void onInit(){

        ROS_INFO_STREAM("Starting DroneSimNodelet");
        nh = getNodeHandle();
        //p_nh = getPrivateNodeHandle();
        nh.getParam("vehicles/num", drone_num);
        drone_sim_ptr = new DroneSim(drone_num, nh);

        main = std::thread(&DroneSim::run, drone_sim_ptr);
    }


    std:: thread main;
    DroneSim* drone_sim_ptr;
    ros::NodeHandle nh;
    //ros::NodeHandle p_nh;
    int drone_num;
};


/*
class DroneSimNodelet: public nodelet::Nodelet{
public:
    DroneSimNodelet() {};
    ~DroneSimNodelet() {};

private:
    virtual void onInit(){

        ROS_INFO_STREAM("Starting DroneSimNodelet");

        ros::NodeHandle nh = getNodeHandle();
        //ros::NodeHandle p_nh = getPrivateNodeHandle();

        int drone_num;
        nh.getParam("vehicles/num", drone_num);

        DroneSim drone_sim(drone_num, nh);

        drone_sim.run();

    }
};
*/
PLUGINLIB_EXPORT_CLASS(DroneSimNodelet, nodelet::Nodelet)