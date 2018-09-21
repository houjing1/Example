/*
 * parameter.h
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_PARAMETER_H
#define NEW_SIMULATOR_PARAMETER_H

#include <Eigen/Dense>
#include <cmath>

//! The Parameter class.
/*!
 * Parameter class contains constant drone’s physical parameters such as inertia matrix, mass,
 * thrust and drag coefficients, Euler angle limits and etc.
 * It also contains the information about different loop frequencies such as the update rate of the position controller and etc.
 * This is the class that will be used by most of the other classes to retrieve relevant parameters.
 */
class Parameter{

public:
    /*** Constructor ***/

    //! Default constructor
    Parameter();

    /*** Memebers ***/

    // drone related
    const double m                            = 0.033;            /*!< m, mass of vehicle (kg) */
    const double Ix                           = 0.00001395;       /*!< Ix, inertia around the body's x-axis (kg-m^2) */
    const double Iy                           = 0.00001436;       /*!< Iy, inertia around the body's y-axis (kg-m^2) */
    const double Iz                           = 0.00002173;       /*!< Iz, inertia around the body's z-axis (kg-m^2) */

    const double g                            = 9.81;             /*!< g, mass normalized gravitational force (m/s^2) */
    const double L                            = 0.03973;          /*!< L, vehicle arm length (m) */
    const double kT                           = 0.2025;           /*!< kT, Non-dimensional thrust coefficient */
    const double kD                           = 0.11;             /*!< kD, Non-dimensional torque coefficient */
    const double r                            = 0.0231348;        /*!< r, Rotor radius (m) */
    const double Ct                           = 3.1582 * 1e-10;   /*!< Thrust coefficient Ct = kT * air_density * (2r)^4 / 3600 */
    const double Cd                           = 7.9379 * 1e-12;   /*!< Torque coefficient Cd */

    // noise related
    const double noise                        = 0.001;            /*!< estimated standard deviation of Vicon measurements */
    Eigen::Matrix3d acc_cov;                                      /*!< 3 by 3 covariance matrix of acceleration */
    Eigen::Vector3d acc_mean                  = {0.0, 0.0, 0.0};  /*!< 3 by 1 mean vector of acceleration */

    Eigen::Matrix3d vicon_cov;                                    /*!< 3 by 3 covariance matrix of vicon measurement */
    Eigen::Vector3d vicon_mean                = {0.0, 0.0, 0.0};  /*!< 3 by 1 mean vector of vicon measurement*/

    Eigen::Matrix3d angle_cov;                                    /*!< 3 by 3 covariance matrix of euler angle for random walk disturbance force*/
    Eigen::Vector3d angle_mean                = {0.0, 0.0, 0.0};  /*!< 3 by 1 mean vector of euler angle for random walk disturbance force*/

    // control loop frequency
    const double inner_loop_cycle             = 2000.0;           /*!< Update rate of inner loop (us) */
    const double outer_loop_cycle             = 2000.0;           /*!< Update rate of outer loop (us) */
    const double position_loop_cycle          = 10000.0;          /*!< Update rate of outer loop (us) */
    const double mellinger_cycle              = 2000.0;           /*!< Update rate of Mellinger controller (us) */

    // angle related
    const double rpLimit                      = 20.0;             /*!< Maximum roll/pitch angle permitted */
    const double rpLimitOverhead              = 1.10;             /*!< Roll/pitch limit overhead */

    // velocity related
    const double xyVelMax                     = 1.0;              /*!< Maximum xy velocity */
    const double zVelMax                      = 1.0;              /*!< Maximum z velocity */
    const double velMaxOverhead               = 1.10;             /*!< Velocity overhead */

    // Motor Dynamics
    const double a1                           = 0.2685;           /*!< coefficient for mapping from pwm to rpm */
    const double a2                           = 4070.3;           /*!< coefficient for mapping from pwm to rpm */

    // thrust related
    //const double thrustScale                  = 1000.0;           /*!< thrust scale factor from firmware*/
    //const double thrustBase                   = 41000.0;          /*!< base thrust(rpm) from firmware*/
    const double thrustMin                    = 20000.0;                  /*!< Minimum thrust */
    const double thrustLimit                  = 65535.0;                  /*!< thrust limit */
    const double we                           = sqrt((m*g) / (4*Ct));     /*!< equilibrium motor rpm */
    const double thrustScale                  = m / (2*we) / (4*Ct) / a1; /*!< thrust scale factor */
    const double thrustBase                   = (we - a2) / a1;           /*!< base thrust(rpm) */

    // Moment related
    //const double f1 = Ix*sqrt(2.0)/(2*we)/(4*L*Ct)/a1;
    //const double f2 = Iy*sqrt(2.0)/(2*we)/(4*L*Ct)/a1;
    //const double f3 = Ix / (2*we) / (4*Cd) / a1/2.0;

    // Marker parameters
    const Eigen::Vector3d positive_x          = {0.03, 0, 0};     /*!< positive x direction marker distance from COM */
    const Eigen::Vector3d negative_x          = {-0.02, 0, 0};    /*!< negative x direction marker distance from COM */
    const Eigen::Vector3d positive_y          = {0, 0.01, 0};     /*!< positive y direction marker distance from COM */

    Eigen::Matrix3d inertia;                                      /*!< 3 by 3 2nd moment of inertia about COM */
    Eigen::Matrix3d inertia_inv;                                  /*!< inverse of 3 by 3 2nd moment of inertia about COM */
    Eigen::Matrix<double, 1, 3> torque_coeff;                     /*!< 1 by 3 row matrix of torque coefficients */
    Eigen::Matrix<double, 3, 4> rpm_to_torque;                    /*!< 3 by 4 transformation matrix to map motor rpm to torque about COM */

    // collision
    const double ellipsoid_a                  = 0.2;              /*!< ellipsoid equation parameter */
    const double ellipsoid_b                  = 0.2;              /*!< ellipsoid equation parameter */
    const double ellipsoid_c                  = 0.75;             /*!< ellipsoid equation parameter */
    Eigen::Matrix3d collision_matrix;                             /*!< diagonal matrix made up with ellipsoid parameters */

};

// constructor
inline
Parameter::Parameter(){

    inertia = Eigen::Matrix3d::Identity();
    inertia(0,0) = Ix;
    inertia(1,1) = Iy;
    inertia(2,2) = Iz;

    inertia_inv = Eigen::Matrix3d::Identity();
    inertia_inv(0,0) = 1/Ix;
    inertia_inv(1,1) = 1/Iy;
    inertia_inv(2,2) = 1/Iz;

    collision_matrix = Eigen::Matrix3d::Identity();
    collision_matrix(0,0) = 1/ellipsoid_a;
    collision_matrix(1,1) = 1/ellipsoid_b;
    collision_matrix(2,2) = 1/ellipsoid_c;

    torque_coeff(0,0) = L / std::sqrt(2.0) * Ct;
    torque_coeff(0,1) = L / std::sqrt(2.0) * Ct;
    torque_coeff(0,2) = Cd;

    rpm_to_torque << -torque_coeff(0,0), -torque_coeff(0,0),  torque_coeff(0,0),  torque_coeff(0,0),
            -torque_coeff(0,1),  torque_coeff(0,1),  torque_coeff(0,1), -torque_coeff(0,1),
            -torque_coeff(0,2),  torque_coeff(0,2), -torque_coeff(0,2),  torque_coeff(0,2);

    acc_cov << 5.0e-04, 0.0,  0.0,
            0.0,  5.0e-04, 0.0,
            0.0, 0.0,  5.0e-04;

    vicon_cov << 9.97872508e-09,     -1.36612165e-08,     5.14408427e-09,
            -1.36612165e-08, 2.91993200e-08,     -8.75911004e-09,
            5.14408427e-09,     -8.75911004e-09, 1.12466881e-08;

    angle_cov << 0.0, 0.0,  0.0,
            0.0,  0.0, 0.0,
            0.0, 0.0,  5.0e-04;
}


#endif //NEW_SIMULATOR_PARAMETER_H
