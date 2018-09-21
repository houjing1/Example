/*
 * attitudeController.h
 *
 *  Created On : 27/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_ATTITUDECONTROLLER_H
#define NEW_SIMULATOR_ATTITUDECONTROLLER_H

#include <Eigen/Dense>
#include "crazyflie_sim/pid.h"
#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/droneSimInfo.h"
#include "crazyflie_sim/baseController.h"

//! The AttitudeController class.
/*!
 * Attitude controller contains sxi pid controllers, three of them are from the outer loop angle pid controller
 * and the rest three of them are from the inner loop angular velocity pid controller.
 * Thus the AttitudeController class contains six Pid private objects as well as a private Parameter object for relevant control loop parameters.
 * This class provide functionalities to perform update on the inner and outer loop of the controller
 * as well as a total overall update on the attitude controller. It also has the functionality to reset all 6 pid objects.
 * AttitudeController class is derived from the BaseController class.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object.
 */
class AttitudeController : public BaseController {

public:

    /****Constructors****/

    //! constructor function that initializes id, param and pid objects
    AttitudeController(const int& id_value, const DroneSimInfo& info);

    /****Member functions****/

    // Accessor functions
    const int& get_id() const {return id;}                                             /*!< get id number */
    const Eigen::Vector3d& get_pqr_desired() const {return pqr_desired;}               /*!< retrieve pqr_desired attribute as [x, y, z] vector */
    const Eigen::Vector3d& get_motor_var_desired() const {return motor_var_desired;}   /*!< retrieve motor_var_desired attribute as [x, y, z] vector */

    // Utility functions

    /*!
     * \brief update angle pid controller
     *
     * Perform update on the angle sub-controller and return desired angular velocity vector.
     *
     * \param rpy_desired desired euler angle set
     * \param rpy_measured measured euler angle set
     * \return the angular velocity vector as output signal
     */
    const Eigen::Vector3d& update_angle_pid(const Eigen::Vector3d& rpy_desired, const Eigen::Vector3d& rpy_measured);

    /*!
     * \brief update angular velocity pid controller
     *
     * Perform update on the angular velocity sub-controller and return desired motor input variation.
     *
     * \param pqr_desired desired angular velocity vector
     * \param pqr_measured measured angular velocity vector
     * \return the motor input variation as output signal
     */
    const Eigen::Vector3d& update_angular_rate_pid(const Eigen::Vector3d& pqr_desired, const Eigen::Vector3d& pqr_measured);

    /*!
     * \brief update overall controller
     *
     * Perform update on the entire controller and return output vector as desired motor input variation.
     * This function overrides its base class function.
     *
     * \param setpoint is a Setpoint object that consists of desired euler angle(deg) [desired_rpy]
     * \param state is a State object that consists of measured euler angle(rad) and angular velocity(rad/s) [measured_rpy measured_pqr]
     * \return base class Control object that contains desired motor input variation(pwm)
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

    int id = 0;                                                                 /*!<  object id number */
    static Parameter param;                                                     /*!<  Parameter object */

    Pid roll_PID;                                                               /*!<  roll PID */
    Pid pitch_PID;                                                              /*!<  pitch PID */
    Pid yaw_PID;                                                                /*!<  yaw PID */
    Pid p_PID;                                                                  /*!<  x angular velocity PID */
    Pid q_PID;                                                                  /*!<  y angular velocity PID */
    Pid r_PID;                                                                  /*!<  z angular velocity PID */

    Eigen::Vector3d pqr_desired = Eigen::Vector3d::Zero();                      /*!<  angle PID output(x,y,z component of angular velocity) as [x, y, z] vector */
    Eigen::Vector3d motor_var_desired = Eigen::Vector3d::Zero();                /*!<  angular rate PID output(x,y,z component of motor input variation) as [x, y, z] vector */

    //! initialize controllers
    void set_up_controller(const int& id_value, const DroneSimInfo& info);
};


#endif //NEW_SIMULATOR_ATTITUDECONTROLLER_H
