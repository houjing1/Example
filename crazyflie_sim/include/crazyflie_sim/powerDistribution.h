/*
 * powerDistribution.h
 *
 *  Created On : 28/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_POWERDISTRIBUTION_H
#define NEW_SIMULATOR_POWERDISTRIBUTION_H

#include "crazyflie_sim/parameter.h"
#include <Eigen/Dense>

//! The PowerDistribution class.
/*!
 * PowerDistribution class provides the functionality to calculate desired motor pwm from motor input variation and base motor thrust.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object and perform matrix operations on those objects.
 */
class PowerDistribution{

public:

    /****Constructors****/

    //! initialize and set up data attributes
    explicit PowerDistribution(const int& id_value):id(id_value){ thrust_limit = param.thrustLimit;}

    /****Member functions****/
    // Accessor functions
    const int& get_id() const {return id;}                                                              /*!< get id number */
    const double& get_thrust_limit() const {return thrust_limit;}                                       /*!< retrieve thrust_limit attribute */
    const Eigen::Vector4d& get_motor_pwm() const {return motor_pwm;}                                    /*!< retrieve motor_pwm attribute */


    // Mutator functions
    PowerDistribution& set_thrust_limit(const double& value){thrust_limit = value; return *this;}       /*!< set thrust_limit attribute */

    // Utility functions

    /*!
     * \brief compute motor pwm
     *
     * Calculate and return the desired motor pwm value
     *
     * \param motor_variation motor_variation as a 3 by 1 [x,y,z] vector
     * \param base_thrust motor base thrust value
     * \return the motor pwm value
     */
    const Eigen::Vector4d& compute_motor_pwm(const Eigen::Vector3d& motor_variation, const double& base_thrust);

private:

    int id = 0;                                                                                 /*!<  object id number */
    static Parameter param;                                                                     /*!<  Parameter object */
    double thrust_limit = 0.0;                                                                  /*!<  thrust limit */
    Eigen::Vector4d motor_pwm = Eigen::Vector4d::Zero();                                        /*!<  desired motor pwm vector as a 4 by 1 [motor1, motor2, motor3, motor4] vector */

    Eigen::Matrix<double, 4, 3> data_matrix = Eigen::Matrix<double, 4, 3>::Zero();              /*!<  matrix of past motor pwm data */
};

#endif //NEW_SIMULATOR_POWERDISTRIBUTION_H
