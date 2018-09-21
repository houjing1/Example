/*
 * quadrotor.h
 *
 *  Created On : 29/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_QUADROTOR_H
#define NEW_SIMULATOR_QUADROTOR_H

#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <string>
#include "crazyflie_sim/droneSimInfo.h"
#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/state.h"
#include "crazyflie_sim/positionController.h"
#include "crazyflie_sim/attitudeController.h"
#include "crazyflie_sim/positionController_m.h"
#include "crazyflie_sim/attitudeController_m.h"
#include "crazyflie_sim/powerDistribution.h"
#include "crazyflie_sim/motorDynamics.h"
#include "crazyflie_sim/quadrotorDynamics.h"
#include "crazyflie_sim/droneCmd.h"

//! The Quadrotor class.
/*!
 * QuadrotorDynamics class provides the functionality to take motor rpm values
 * and update the drone’s current state by going through drone dynamics.
 * It contain a State object to retain the latest state from each update as a data member
 * as well as a Parameter object for acquiring relevant drone parameters and update frequencies.
 * Any disturbance forces could also be included into the update by passing a list of callable function
 * that could generate disturbance force with a State object as input as an additional argument to the update function.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object and perform matrix operations on those objects.
 */
class Quadrotor {

public:
    /** Constructors **/

    /*!
     * \brief constructor for Quadrotor object
     *
     *  Construct a QuadrotorDynamics object with a id value and use DroneSimInfo to initialize its State,
     *  PositionController, AttitudeController, PowerDistribution, MotorDynamics and QuadrotorDyanmics objects.
     *
     *  The att_only has a default value of false.
     *  It is used to indicate that whether the quadrotor would only utilize its attitude controller.
     *  If only the “AltHold” command is expected to be received throughout the entire simulation,
     *  the a “true” is recommended to be passed to allow the update to skip the position controller.
     *
     * \param id_value id value of this drone
     * \param info DroneSimInfo object for initializing other objects
     * \param type is the controller type used for this drone. Default value is "pid". Either "pid" or "Mellinger" is supported
     * \param reference is the source of reference signal used by controller Default value is "true_state" Either "true_state" or "estiamted_state" are supported
     * \param use_multi_markers is the flag to use multiple vicon markers
     * \param disturbance_force is the flag to include disturbance forces
     * \param measurement_noise is the flag to include measurement noise
     * \param att_only flag to indicate whether only "AldHold" command will be reveived. Default value is false
     */
    Quadrotor(const int& id_value,
              const DroneSimInfo& info,
              std::string type = "pid",
              std::string reference = "true_state",
              bool use_multi_markers = false,
              bool disturbance_force = false,
              bool measurement_noise = false,
              bool att_only = false);

    /** Member functions **/

    // Accessor functions

    const int& get_id() const {return id;}                              /*!< get id member */
    double get_thrust_actuator();                                       /*!< thread safe version of getting thrust_actuator member */
    State get_state();                                                  /*!< thread safe version of getting true_state member */
    State get_estimated_state();                                        /*!< thread safe version of getting estimated_state member */
    Eigen::Vector3d get_command_euler();                                /*!< thread safe version of getting command_euler member */
    Eigen::Vector3d get_motor_variation();                              /*!< thread safe version of getting motor_variation member */
    Eigen::Vector4d get_motor_pwm();                                    /*!< thread safe version of getting motor_pwm member */
    Eigen::Vector4d get_motor_rpm();                                    /*!< thread safe version of getting motor_rpm member */
    DroneCmd get_latest_cmd();                                          /*!< thread safe version of getting latest_cmd member */
    std::string get_previous_cmd_mode();                                /*!< thread safe version of getting previous_cmd_mode member */
    std::string get_controller_type();                                  /*!< get controller_type member*/

    // overloaded accessor functions

    void get_thrust_actuator(double&);                                  /*!< thread safe version of getting thrust_actuator member */
    void get_state(State&);                                             /*!< thread safe version of getting true_state member */
    void get_estiamted_state(State&);                                   /*!< thread safe version of getting estiamted_state member */
    void get_command_euler(Eigen::Vector3d&);                           /*!< thread safe version of getting command_euler member */
    void get_motor_variation(Eigen::Vector3d&);                         /*!< thread safe version of getting motor_variation member */
    void get_motor_pwm(Eigen::Vector4d&);                               /*!< thread safe version of getting motor_pwm member */
    void get_motor_rpm(Eigen::Vector4d&);                               /*!< thread safe version of getting motor_rpm member */
    void get_latest_cmd(DroneCmd&);                                     /*!< thread safe version of getting latest_cmd member */
    void get_previous_cmd_mode(std::string&);                           /*!< thread safe version of getting previous_cmd_mode member */
    void get_controller_type(std::string&);                             /*!< get controller_type member*/

    // Mutator functions
    void set_latest_cmd(const DroneCmd&);                               /*!< thread safe version of setting latest_cmd member */
    void set_controller_type(const std::string&);                       /*!<set controller_type member*/
    void set_estimated_state(const State&);                             /*!< thread safe version of setting estimated_state member*/
    void set_thrust_actuator(const double&);                            /*!< thread safe version of setting thrust_actuator member */
    void set_command_euler(const Eigen::Vector3d&);                     /*!< thread safe version of setting command_euler member */
    void set_motor_variation(const Eigen::Vector3d&);                   /*!< thread safe version of setting motor_variation member */
    void set_motor_pwm(const Eigen::Vector4d&);                         /*!< thread safe version of setting motor_pwm member */
    void set_motor_rpm(const Eigen::Vector4d&);                         /*!< thread safe version of setting motor_rpm member */


    // Utility functions

    /*!
     * \brief updated position controller
     *
     * Perform update on the drone’s position controller. The results are stored in its command_euler and thrust_actuator private data member.
     * This function could either update the pid or Mellinger's position controller
     */
    void update_position_controller();

    /*!
     * \brief updated attitude controller
     *
     * Perform update on the drone’s attitude controller.
     * The results are stored in its motor_variation private data member.
     * This function could either update the pid or Mellinger's attitude controller
     */
    void update_attitude_controller();

    // decomposition of updating attitude_controller

    /*!
     * \brief updated outer loop of attitude controller
     *
     * Perform update on the drone’s attitude controller’s outer loop.
     * This function could be used in place of the void update_attitude_controller()
     * when attitude controller’s outer loop has a different update rate as its inner loop.
     *
     * It only update the Pid controller
     */
    void update_attitude_controller_outer();

    /*!
     * \brief updated inner loop of attitude controller
     *
     * Perform update on the drone’s attitude controller’s inner loop.
     * The results are stored in its motor_variation private data member.
     * This function could be used in place of the void update_attitude_controller()
     * when attitude controller’s inner loop has a different update rate as its outer loop.
     *
     * It only update the Pid controller
     */
    void update_attitude_controller_inner();

    /*!
     * \brief updated drone dynamics
     *
     * Perform update on the drone’s dynamics.
     * The results are stored in its state private data member.
     * Input is a list of functions that could be used to generate disturbance forces.
     *
     * \param external_force a vector of pointer to external force functions
     */
    //void update_motor_output(const std::vector<QuadrotorDynamics::ptr_func>& external_force = std::vector<QuadrotorDynamics::ptr_func>());
    void update_motor_output();

    // generate position vector from state member
    /*!
     * \brief generate position
     *
     * Generate the position of this drone’s center of mass.
     *
     * \return the 3 by 1 position vector
     */
    Eigen::Vector3d generate_position();

    /*!
     * \brief generate position
     *
     * Generate the position of this drone’s center of mass.
     *
     * \param pos reference to a variable will be changed in place
     */
    void generate_position(Eigen::Vector3d& pos);

    /*!
     * \brief generate multiple vicon marker position
     *
     * Generate the position of this drone’s markers. In place change is performed on the input parameter.
     *
     * \param pos_list reference to a variable will be changed in place
     */
    void generate_markers_position(std::vector<Eigen::Vector3d>& pos_list);

    /*!
     * \brief reset controller
     *
     * Reset the drone’s controller. Input is the type of controller to be set. It could either be “position” or “attitude”.
     *
     * \param type controller type
     */
    void reset_controller(const std::string& type);

private:

    int id = 0;                                                                                     /*!< object id number */
    static Parameter param;                                                                         /*!< Parameter object */
    State true_state;                                                                               /*!< State object for current drone's true state */
    State estimated_state;                                                                          /*!< State object for current drone's estimated state */

    bool multi_markers_flag = false;                                                                /*!< flag for using single or multi vicon markers */
    bool attitude_only = false;                                                                     /*!< flag for only using the attitude controller */
    bool disturbance_flag = false;                                                                  /*!< flag for including disturbance forces */
    bool noise_flag = false;                                                                        /*!< flag for including measurement noise */
    std::string controller_type = "pid";                                                            /*!< name of controller type, either "pid" or "Mellinger" */
    std::string reference_source = "true_state";                                                    /*!< type of reference source, either "true_state" or "estiamted_state" */

    QuadrotorDynamics::vector_of_ptr_func external_force = QuadrotorDynamics::vector_of_ptr_func(); /*!< list of pointer to external force functions */

    PositionController position_controller;                                                     /*!< PositionController object */
    AttitudeController attitude_controller;                                                     /*!< AttitudeController object */
    PositionController_m position_controller_m;                                                 /*!< PositionController_m object */
    AttitudeController_m attitude_controller_m;                                                 /*!< AttitudeController_m object */
    PowerDistribution power_distribution;                                                       /*!< PowerDistribution object */
    MotorDynamics motor_dynamics;                                                               /*!< MotorDynamics object */
    QuadrotorDynamics quadrotor_dynamics;                                                       /*!< QuadrotorDynamics object */

    std::string previous_cmd_mode = "NA";                                                       /*!< string object that store the previous drone command mode("PosSet" or "AltHold") */
    DroneCmd latest_cmd;                                                                        /*!< DroneCmd object to store the latest command */
    double thrust_actuator = 0.0;
    Eigen::Vector3d command_euler = Eigen::Vector3d::Zero();                                    /*!< 3 by 1 vector of latest euler angle set points */
    Eigen::Vector3d motor_variation = Eigen::Vector3d::Zero();                                  /*!< 3 by 1 vector of latest motor input variation value */
    Eigen::Vector4d motor_pwm = Eigen::Vector4d::Zero();                                        /*!< 4 by 1 vector of latest mapping from motor input variation to motor pwm */
    Eigen::Vector4d motor_rpm = Eigen::Vector4d::Zero();                                        /*!< 4 by 1 vector of latest mapping from motor pwm to motor rpm */

    mutable std::shared_timed_mutex state_mutex;                                                /*!< mutex object for safely accessing true_state member */
    mutable std::shared_timed_mutex estimate_mutex;                                             /*!< mutex object for safely accessing estimated_state member */
    mutable std::shared_timed_mutex thrust_mutex;                                               /*!< mutex object for safely accessing thrust_actuator member */
    mutable std::shared_timed_mutex euler_mutex;                                                /*!< mutex object for safely accessing command_euler member */
    mutable std::shared_timed_mutex motor_var_mutex;                                            /*!< mutex object for safely accessing motor_variation member */
    mutable std::shared_timed_mutex motor_pwm_mutex;                                            /*!< mutex object for safely accessing motor_pwm member */
    mutable std::shared_timed_mutex motor_rpm_mutex;                                            /*!< mutex object for safely accessing motor_rpm member */
    mutable std::shared_timed_mutex cmd_mutex;                                                  /*!< mutex object for safely accessing latest_cmd member */
    mutable std::shared_timed_mutex position_controller_mutex;                                  /*!< mutex object for safely accessing position controller */
    mutable std::shared_timed_mutex attitude_controller_mutex;                                  /*!< mutex object for safely accessing attitude controller */
    mutable std::shared_timed_mutex mode_mutex;                                                 /*!< mutex object for safely accessing previous_cmd_mode member */

    // Assistant functions

    /*!
     * \brief reset controller
     *
     * Check if command mode has changed and reset corresponding controller states
     */
    void transition_reset();

    //! thread safe version of setting previous_cmd_mode member
    void set_previous_cmd_mode(const std::string& mode);

    /*!
     * \brief shutdown drone
     *
     * Reset the private data members including thrust_actuator, command_euler, motor_variation,
     * motor_pwm, motor_rpm as well as controllers
     */
    void shutdown();

    /*!
     * \brief check if drone is commanded to be near the ground
     *
     * \param cmd is a DroneCmd object that contains the latest command for this drone
     */
    bool near_ground(const DroneCmd& cmd);


};

#endif //NEW_SIMULATOR_QUADROTOR_H