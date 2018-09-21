/*
 * quadrotorDynamics.h
 *
 *  Created On : 28/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_QUADROTORDYNAMICS_H
#define NEW_SIMULATOR_QUADROTORDYNAMICS_H

#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/state.h"
#include <Eigen/Dense>
#include <vector>

//! The QuadrotorDynamics class.
/*!
 * QuadrotorDynamics class provides the functionality to take motor rpm values and update the drone’s current state
 * by going through drone dynamics. It contain a State object to retain the latest state from each update as a data member
 * as well as a Parameter object for acquiring relevant drone parameters and update frequencies.
 * Any disturbance forces could also be included into the update by passing a list of callable function
 * that could generate disturbance force with a State object as input as an additional argument to the update function.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object and perform matrix operations on those objects.
 */
class QuadrotorDynamics {

public:

    /****Constructors****/

    //! Constructor that takes in a id value
    explicit QuadrotorDynamics(const int& id_value):id(id_value), updated_state(id_value){dt = param.inner_loop_cycle * 1.0e-6;}

    /****Member functions****/

    // Accessor functions
    const int& get_id() const {return id;}                                                      /*!< get id number */
    const double& get_dt() const {return dt;}                                                   /*!< get update period(s) */
    const Eigen::Vector4d& get_current_force() const {return current_force;}                    /*!< get current_force attribute */
    const Eigen::Vector3d& get_current_torque() const {return current_torque;}                  /*!< get current_torque attribute */
    const State& get_updated_state() const {return updated_state;}                              /*!< get updated_state attribute */

    // Mutator functions
    QuadrotorDynamics& set_dt(const double& value){dt = value;}                                 /*!< set update period(s) */

    // Utility functions

    //! Typedef.
    /*! alias for pointer to external_force function. */
    using ptr_func = Eigen::Vector3d (*) (const State&);

    //! Typedef.
    /*! alias for vector of pointer to external_force function. */
    using vector_of_ptr_func = std::vector<ptr_func>;

    /*!
     * \brief compute updated state
     *
     * Calculate and return the reference to the current state object which has it states updated by applying drone dynamics equations.
     * This version of update compute the rate of change of rotation matrix
     *
     * \param current_state reference to the current state object
     * \param motor_rpm current motor rpm values
     * \param external_force a vector of pointer to external force functions
     * \return The reference to the current_state object that is passed in as argument
     */
    const State& update_state(State& current_state, const Eigen::Vector4d& motor_rpm, const std::vector<ptr_func>& external_force = std::vector<ptr_func>());

    /*!
     * \brief compute updated state
     *
     * Calculate and return the reference to the current state object which has it states updated by applying drone dynamics equations.
     * This version of update compute the rate of change of euler angles
     *
     * \param current_state reference to the current state object
     * \param motor_rpm current motor rpm values
     * \param external_force a vector of pointer to external force functions
     * \return The reference to the current_state object that is passed in as argument
     */
    const State& update_state2(State& current_state, const Eigen::Vector4d& motor_rpm, const std::vector<ptr_func>& external_force = std::vector<ptr_func>());

    /*!
     * \brief compute updated state
     *
     * Calculate and return the reference to the current state object which has it states updated by applying drone dynamics equations.
     * This version of update compute the rate of change of quaternion
     *
     * \param current_state reference to the current state object
     * \param motor_rpm current motor rpm values
     * \param external_force a vector of pointer to external force functions
     * \return The reference to the current_state object that is passed in as argument
     */
    const State& update_state3(State& current_state, const Eigen::Vector4d& motor_rpm, const std::vector<ptr_func>& external_force = std::vector<ptr_func>());

    /*!
     * \brief compute updated state
     *
     * Calculate and return the reference to the current state object which has it states updated by applying drone dynamics equations.
     * This version of update compute the rate of change of euler angles and is based on the source:
     * http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf
     *
     * \param current_state reference to the current state object
     * \param motor_rpm current motor rpm values
     * \param external_force a vector of pointer to external force functions
     * \return The reference to the current_state object that is passed in as argument
     */
    const State& update_state4(State& current_state, const Eigen::Vector4d& motor_rpm, const std::vector<ptr_func>& external_force = std::vector<ptr_func>());

private:

    int id = 0;                                                                 /*!< object id number */
    static Parameter param;                                                     /*!< Parameter object */
    double dt = 0.0;                                                            /*!< dynamics update period(us) */
    State updated_state;                                                        /*!< updated state of drone as a State object */
    Eigen::Vector4d current_force = Eigen::Vector4d::Zero();                    /*!< current forces generated by each motor as a 4 by 1 vector of [motor1, motor2, motor3, motor4] */
    Eigen::Vector3d current_torque = Eigen::Vector3d::Zero();                   /*!< current torque about COM generated by thrust forces as a 3 by 1 vector of [x, y, z] */

    /*!
     * \brief determine motor forces
     *
     * This function determines the four motor thrust forces from their own rpm
     *
     * \param motor_rpm is the 4 by 1 vector of motor rpm value
     */
    void determine_force(const Eigen::Vector4d& motor_rpm);

    /*!
     * \brief determine torque
     *
     * This function determines torque act on the COM generated by the four motor thrust and drag forces
     *
     * \param motor_rpm is the 4 by 1 vector of motor rpm value
     */
    void determine_torque(const Eigen::Vector4d& motor_rpm);

    /*!
     * \brief constrain states when drone is on the ground
     *
     * This function constrains the states of drone when it is on the ground
     * \param current_state is the current state of the drone
     * \param acc is the latest acceleration calcualted from drone dynamcis
     */
    void grounding(State& current_state, const Eigen::Vector3d& acc);

};


#endif //NEW_SIMULATOR_QUADROTORDYNAMICS_H
