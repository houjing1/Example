/*
 * droneSim.h
 *
 *  Created On : 06/07/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef PROJECT_DRONESIM_H
#define PROJECT_DRONESIM_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "crazyflie_sim/droneSimInfo.h"
#include "crazyflie_sim/droneCmd.h"
#include "crazyflie_estimator/FullState.h"
#include "crazyflie_estimator/SwarmStates.h"

#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/quadrotor.h"
#include "crazyflie_central/SwarmCmds.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dsl__utilities__msg/Marker.h>
#include <dsl__utilities__msg/Markers.h>
#include <gazebo_msgs/ModelState.h>
#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <vector>
#include <mutex>
#include <memory>
#include <std_msgs/Bool.h>
#include <string>

//! The DroneSim class.
/*!
 * DroneSim Class provides the final encapsulation of the simulator by containing N number of Quadrotor objects,
 * where N is the number of drones to simulate, a Parameter object for loop frequency information,
 * a DroneSimInfo object as well as the classes from ros C++ client library to interact with ROS.
 * It populate the DroneSimInfo object with the parameters extracted from ROS parameter server
 * and will then use it to initialize each Quadrotor object.
 * It provide functionalities to extract and parse command messages into DroneCmd object and distribute them to each Quadrotor object,
 * to ask each Quadrotor to update tis controllers and dynamics at a specified rate,
 * and to generate messages of vion markers from all of the drones by requesting each drone to provide its latest position.
 */
class DroneSim{

public:

    /*!
     * \brief his is the constructor for DroneSim.
     *
     * Upon constructing the object, different initialization process happen in sequence as the following:
     * 1. set up additional node handles and callback queues
     * 2. update the DroneSimInfo object
     * 3. initialize gazebo messages
     * 4. create a list of Quadrotor objects
     * 5. set up subscriber
     * 6. set up publishers
     * 7. set up timers
     */
    DroneSim(const int& drone_num, ros::NodeHandle& handle);

    /*!
     * \brief Member function to run this node.
     *
     * Run this ROS node by allowing each node handle to process their own callback queues.
     */
    void run();

private:

    // Member data //

    int counter = 0;                                                                    /*!< number of first updates to ignore(for stability issues) */
    double euler_angle_max = 0.5236; /** need this? **/                                 /*!< limit of euler angle command in rad(30 deg)*/

    // flags
    bool use_gazebo = false;                                                            /*!< flag to indicate if gazebo simulator is used */
    bool use_multi_markers = false;                                                     /*!< flag to indicate if multiple markers are used for vicon message */
    bool random_shuffle = false;                                                        /*!< flag to indicate if markers will be randomly shuffled before being published */
    bool detect_collision = false;                                                      /*!< flag to indicate if or not to detect collision*/

    // data and info
    ros::NodeHandle nh;                                                                 /*!< ROS node handle for general messages */
    ros::NodeHandle nh_slow;                                                            /*!< ROS node handle for slow control loop */
    ros::NodeHandle nh_fast;                                                            /*!< ROS node handle for fast control loop */

    ros::CallbackQueue queue_general;                                                   /*!< ROS custom callback queue for general messages */
    ros::CallbackQueue queue_slow;                                                      /*!< ROS custom callback queue for slow controller loop */
    ros::CallbackQueue queue_fast;                                                      /*!< ROS custom callback queue for fast controller loop */

    ros::AsyncSpinner spinner_general;                                                  /*!< allocate separate thread for handling general queue */
    ros::AsyncSpinner spinner_slow;                                                     /*!< allocate separate thread for handling slow queue */
    ros::AsyncSpinner spinner_fast;                                                     /*!< allocate separate thread for handling fast queue */

    Parameter param;                                                                    /*!< Parameter object which include drone-related parameters */
    DroneSimInfo info;                                                                  /*!< DroneSimInfo object which include pid-related parameters */
    int num_of_drone = 0;                                                               /*!< number of drones configured */

    std::vector<std::shared_ptr<Quadrotor>> drones;                                     /*!< vector of pointers to  Quadrotor objects */
    dsl__utilities__msg::MarkersPtr vicon_msg_ptr;                                      /*!< shared pointer of position message for estimator */

    std::vector<gazebo_msgs::ModelState> gazebo_msg_sim;                                /*!< vector of StateModel objects */
    std::vector<gazebo_msgs::ModelState> gazebo_msg_des;                                /*!< vector of StateModel objects */
    std::vector<Eigen::Vector3d> pos_list;                                              /*!< vector of marker positions for multiple marks */

    // publishers and subscribers
    ros::Subscriber subscriber_cmd;                                                     /*!< ROS Subscriber object to subscribe command message */
    ros::Subscriber subscriber_state;                                                   /*!< ROS Subscriber object to subscribe full state message */
    ros::Publisher publisher_marker;                                                    /*!< ROS Publisher object to publish vicon_msg to Estimator */
    ros::Publisher publisher_gazebo;                                                    /*!< ROS Publisher object to publish messages to Gazebo simulator */
    ros::Publisher test_publisher;                                                      /*!< ROS Publisher object to test topic frequenct */
    ros::Publisher _pub_swarmstates;                                                    /*!< ROS Publisher object to publish drone full states */

    // timers
    ros::Timer position_loop_timer;                                                     /*!< ROS Timer object to perform update on position controller */
    ros::Timer attitude_loop_timer;                                                     /*!< ROS Timer object to perform update on attitude controller and drone dynamics */
    ros::Timer publish_marker_timer;                                                    /*!< ROS Timer object to perform publication of position messages to Estimator */
    ros::Timer outer_attitude_loop_timer;                                               /*!< ROS Timer object to perform update on angle controller */
    ros::Timer inner_attitude_loop_timer;                                               /*!< ROS Timer object to perform update angular rate controller and drone dynamics */
    ros::Timer gazebo_timer;                                                            /*!< ROS Timer object to perform publication of gazebo messages at specific rates */

    // Member functions //

    // initialization functions
    void initialize_drone_list();                                                       /*!< function to initialize a vector of Quadrotor objects */
    void initialize_vicon_msg();                                                        /*!< function to initialize a Markers object as publisher message for Estimator */
    void initialize_gazebo_msg();                                                       /*!< function to initialize vectors of ModelState objects */

    void initialize_subscribers();                                                      /*!< function to initialize subscribers */
    void initialize_publishers();                                                       /*!< function to initialize publishers */
    void initialize_timers();                                                           /*!< function to initialize timers */

    // callback functions
    void cmd_subscriber_callback(const crazyflie_central::SwarmCmdsPtr& cmd_ptr);           /*!< callback function for cmd subscribers */
    void state_subscriber_callback(const crazyflie_estimator::SwarmStatesPtr& state_ptr);   /*!< callback function for cmd subscribers */
    void position_loop_timer_callback(const ros::TimerEvent&);                              /*!< callback function for position loop timer */
    void attitude_loop_timer_callback(const ros::TimerEvent&);                              /*!< callback function for inner loop timer */
    void combined_loop_timer_callback(const ros::TimerEvent&);                              /*!< callback function for combined position and attitude loop timer */
    void publish_marker_timer_callback(const ros::TimerEvent&);                             /*!< callback function for publish_marker_timer */
    void outer_attitude_loop_timer_callback(const ros::TimerEvent&);                        /*!< callback function for outer_attitude_loop_timer */
    void inner_attitude_loop_timer_callback(const ros::TimerEvent&);                        /*!< callback function for inner_attitude_loop_timer */
    void gazebo_timer_callback(const ros::TimerEvent&);                                     /*!< callback function for gazebo_timer */

    // Assistant functions
    bool setup_DroneSimInfo();                                                          /*!< extract values from ROS parameter server as store in DroneSimInfo object */
    bool get_pid_param(XmlRpc::XmlRpcValue& pid_param ,Eigen::Vector4d& pid_vector);    /*!< helping function used by setup_DroneSimInfo */
    bool check_collision();                                                             /*!< function to check if two drones has collided */

};

#endif //PROJECT_DRONESIM_H
