/*
 * positionController.h
 *
 *  Created On : 27/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_POSITIONCONTROLLER_H
#define NEW_SIMULATOR_POSITIONCONTROLLER_H

#include <Eigen/Dense>
#include "crazyflie_sim/pid.h"
#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/droneSimInfo.h"
#include "crazyflie_sim/baseController.h"

//! The PositionController class.
/*!
 * Position controller contains sxi pid controllers, three of them are from the outer loop position pid controller
 * and the rest three of them are from the inner loop velocity pid controller.
 * Thus the PostionController class contains six Pid private objects as well as a private Parameter object for relevant control loop parameters.
 * This class provide functionalities to perform update on the inner and outer loop of the controller
 * as well as a total overall update on the position controller.
 * It also has the functionality to reset all 6 pid objects. PositionController class is derived from the BaseController class.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object.
 */
class PositionController : public BaseController {

public:

    /****Constructors****/

    //! constructor function that initializes id, param and pid objects
    PositionController(const int& id_value, const DroneSimInfo& info);

    /****Member functions****/

    // Accessor functions
    const int& get_id() const {return id;}                                             /*!< get id number */
    const Eigen::Vector3d& get_vel_desired() const {return vel_desired;}               /*!< retrieve vel_desired attribute as [x, y, z] vector */
    const Eigen::Vector3d& get_acc_desired() const {return acc_desired;}               /*!< retrieve acc_desired attribute as [x, y, z] vector */
    const Eigen::Vector3d& get_trp_desired() const {return trp_desired;}               /*!< retrieve trp_desired attribute as [thrust, roll, pitch] vector */

    // Utility functions

    /*!
     * \brief update position pid controller
     *
     * Perform update on the position sub-controller and return desired velocity vector.
     *
     * \param pos_desired desired position vector
     * \param pos_measured measured position vector
     * \return the vel_desired vector as output signal
     */
    const Eigen::Vector3d& update_position_pid(const Eigen::Vector3d& pos_desired, const Eigen::Vector3d& pos_measured);

    /*!
     * \brief update velocity pid controller
     *
     * Perform update on the velocity sub-controller and return desired thrust-roll-pitch vector.
     *
     * \param pos_desired desired velocity vector
     * \param pos_measured measured velocity vector
     * \return the trp_desired vector as output signal
     */
    const Eigen::Vector3d& update_velocity_pid(const Eigen::Vector3d& vel_desired, const Eigen::Vector3d& vel_measured, const double& yaw_measured);

    /*!
     * \brief update overall controller
     *
     * Perform update on the entire controller and return output vector as desired thrust-roll-pitch.
     * This function overrides its base class function.
     *
     * \param setpoint is a Setpoint object that consists of desired position
     * \param state is a State object that consists of measured position, velocity as well as the measured yaw angle
     * \return base class Control object that contains desired thrust, roll, pitch
     */
    const Control& update_overall(const Setpoint& setpoint, const State& state) override;

    /*!
     * \brief reset controller
     *
     * Reset controller internal states.
     * This function overrides its base class function.
     */
    void reset_controller() override;


private:

    int id = 0;                                                                 /*!< object id number */
    static Parameter param;                                                     /*!< Parameter object */

    Pid x_PID;                                                                  /*!< x position PID */
    Pid y_PID;                                                                  /*!< y position PID */
    Pid z_PID;                                                                  /*!< z position PID */
    Pid vx_PID;                                                                 /*!< x velocity PID */
    Pid vy_PID;                                                                 /*!< y velocity PID */
    Pid vz_PID;                                                                 /*!< z velocity PID */

    Eigen::Vector3d vel_desired = Eigen::Vector3d::Zero();                      /*!< position PID output as [x, y, z] vector */
    Eigen::Vector3d acc_desired = Eigen::Vector3d::Zero();                      /*!< velocity PID intermediate output as [x, y, z] vector */
    Eigen::Vector3d trp_desired = Eigen::Vector3d::Zero();                      /*!< velocity PID output as [thrust, roll, pitch] vector */

    //! initialize controllers
    void set_up_controller(const int& id_value, const DroneSimInfo& info);
};


#endif //NEW_SIMULATOR_POSITIONCONTROLLER_H
