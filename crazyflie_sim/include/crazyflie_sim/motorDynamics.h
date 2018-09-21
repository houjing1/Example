/*
 * motorDynamics.h
 *
 *  Created On : 28/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_MOTORDYNAMICS_H
#define NEW_SIMULATOR_MOTORDYNAMICS_H

#include "crazyflie_sim/parameter.h"
#include <Eigen/Dense>

//! The MotorDyamics class.
/*!
 * MotorDynamics class provides the functionality to map motor pwm to motor rpm.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object and perform matrix operations on those objects.
 */
class MotorDynamics {

public:

    /****Constructors****/

    //! initialize and set up data attributes
    explicit MotorDynamics(const int& id_value):id(id_value){}

    /****Member functions****/
    // Accessor functions

    const int& get_id() const {return id;}                                                      /*!< get id number */
    const Eigen::Vector4d& get_motor_rpm() const {return motor_rpm;}                            /*!< retrieve motor_rpm attribute */

    // Utility functions

    /*!
     * \brief compute motor rpm
     *
     * Calculate and return the desired motor rpm value
     *
     * \param motor_pwm motor pwm value as 4 by 1 [motor1,motor2,motor3,motor4] vector
     * \return the motor rpm value
     */
    const Eigen::Vector4d& compute_motor_rpm(const Eigen::Vector4d& motor_pwm);

private:

    int id = 0;                                                                                 /*!< object id number */
    static Parameter param;                                                                     /*!< Parameter object */
    Eigen::Vector4d motor_rpm = Eigen::Vector4d::Zero();                                        /*!< desired motor rpm(revolution per minute) as a 4 by 1 [motor1, motor2, motor3, motor4] vector */

};


#endif //NEW_SIMULATOR_MOTORDYNAMICS_H
