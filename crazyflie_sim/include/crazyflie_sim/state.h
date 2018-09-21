/*
 * state.h
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_STATE_H
#define NEW_SIMULATOR_STATE_H

#include <Eigen/Dense>
#include <vector>
#include "crazyflie_sim/droneSimInfo.h"

//! The State class.
/*!
 * State class contains the current state information of a single drone.
 * It contains the drone’s position, velocity, acceleration, rotation matrix,
 * Euler angle and so on. It also provide functionality to convert among rotation matrix,
 * Euler angle and quaternions.
 */
class State{

public:

    /****Constructors****/

    //! Default constructor
    State() = default;

    /*!
     * \brief Alternative constructor
     *
     * \param id_value The id number for this State object
     */
    explicit State(const int& id_value);

    /*!
     * \brief Alternative constructor
     *
     * \param id_value The id number for this State object
     * \param info The DroneSimInfo object for initial position information
     */
    State(const int& id_value, const DroneSimInfo& info);

    /*!
     * \brief Alternative constructor
     *
     * \param id_value The id number for this State object
     * \param x_init The initial x position
     * \param y_init The initial y position
     */
    State(const int& id_value, const double& x_init, const double& y_init);


    /****Member functions****/

    // Accessor functions
    const Eigen::Matrix3d& get_R() const { return R; }                      /*!< Accessor for rotation matrix */
    const Eigen::Vector3d& get_position() const { return pos; }             /*!< Accessor for position vector */
    const Eigen::Vector3d& get_velocity() const { return vel; }             /*!< Accessor for velocity vector */
    const Eigen::Vector3d& get_acceleration() const { return acc; }         /*!< Accessor for acceleration vector */
    const Eigen::Vector3d& get_omega() const { return omega; }              /*!< Accessor for angular velocity vector */
    const Eigen::Vector3d& get_euler() const { return euler; }              /*!< Accessor for euler angle set */
    const Eigen::Vector3d& get_euler_rate() const { return euler_rate; }    /*!< Accessor for euler angle rate set */
    const Eigen::Quaterniond& get_quaternion() const { return quaternion; } /*!< Accessor for euler angle set */

    const int& get_id() const { return id; }                                /*!< Accessor for id number */

    // Mutator functions
    State& set_R(const Eigen::Matrix3d& R_value){ R = R_value; return *this; }                                              /*!< Mutator for rotation matrix */
    State& set_position(const Eigen::Vector3d& pos_value){ pos = pos_value; return *this; }                                 /*!< Mutator for position vector */
    State& set_velocity(const Eigen::Vector3d& vel_value){ vel = vel_value; return *this; }                                 /*!< Mutator for velocity vector */
    State& set_acceleration(const Eigen::Vector3d& acc_value){ acc = acc_value; return *this; }                             /*!< Mutator for acceleration vector */
    State& set_omega(const Eigen::Vector3d& omega_value){ omega = omega_value; return *this; }                              /*!< Mutator for angular velocity vector */
    State& set_euler(const Eigen::Vector3d& euler_value){ euler = euler_value; return *this; }                              /*!< Mutator for euler angle set */
    State& set_euler_rate(const Eigen::Vector3d& euler_rate_value){ euler_rate = euler_rate_value; return *this; }          /*!< Mutator for euler angle rate set */
    State& set_quaternion(const Eigen::Quaterniond& quaternion_value){ quaternion = quaternion_value; return *this; }       /*!< Mutator for quaternion */

    //! Alternative mutator for rotation matrix
    State& set_R(const int& row, const int& col, const double& value){
        check_index(row, col, 3, 3);
        R(row, col) = value;
        return *this;
    }
    //! Alternative mutator for position vector
    State& set_position(const int& index, const double& value){
        check_index(index, 3);
        pos(index) = value;
        return *this;
    }
    //! Alternative mutator for velocity vector
    State& set_velocity(const int& index, const double& value){
        check_index(index, 3);
        vel(index) = value;
        return *this;
    }
    //! Alternative mutator for acceleration vector
    State& set_acceleration(const int& index, const double& value){
        check_index(index, 3);
        acc(index) = value;
        return *this;
    }
    //! Alternative mutator for angular velocity vector
    State& set_omega(const int& index, const double& value){
        check_index(index, 3);
        omega(index) = value;
        return *this;
    }
    //! Alternative mutator for euler angle set
    State& set_euler(const int& index, const double& value){
        check_index(index, 3);
        euler(index) = value;
        return *this;
    }

    //! Alternative mutator for euler angle rate set
    State& set_euler_rate(const int& index, const double& value){
        check_index(index, 3);
        euler_rate(index) = value;
        return *this;
    }

    //! Mutator for id number
    State& set_id(const int& id_value){ id = id_value; return *this; }


    // Utitlity functions

    /*!
     * \brief Convert rotation matrix to euler angle
     *
     * This functions take the rotation matrix R_wb and converts to euler angle set following the 3-2-1 convention
     * \return The converted euler angle set
     */
    Eigen::Vector3d get_euler_from_R() const;

    /*!
     * \brief Convert rotation matrix to quaternion
     *
     * This functions take the rotation matrix R_wb and converts to quaternion
     * \return The converted quaternion
     */
    Eigen::Vector4d get_quaternion_from_R() const;

    /*!
     * \brief Convert euler angle to rotation matrix
     *
     * This functions take the euler angle set following the 3-2-1 convention and convert it to rotation matrix R_wb
     * \return The converted rotation matrix R_wb
     */
    Eigen::Matrix3d get_R_from_euler() const;

    //! reset the state
    void reset();

private:

    int id = 0;                                                 /*!<  id number */

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();            /*!<  3 by 3 rotation matrix, R_wb */
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();              /*!<  3 by 1 position vector */
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();              /*!<  3 by 1 velocity vector */
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();              /*!<  3 by 1 acceleration vector */
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();            /*!<  3 by 1 inertial angular velocity as viewed in body frame (rad/s) */
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();            /*!<  3 by 1 column matrix of roll, pitch, yaw (rad) */
    Eigen::Vector3d euler_rate = Eigen::Vector3d::Zero();       /*!<  3 by 1 column matrix of roll, pitch, yaw rate(rad/s) */
    Eigen::Quaterniond quaternion;                              /*!<  quaternion */

    /*!
     * \brief Check index
     *
     * This functions checks if the given index is out of range
     * \param index reference to index number
     * \param dimension reference to the index dimension
     */
    void check_index( const int& index, const int& dimension) const;

    /*!
     * \brief Check index
     *
     * This functions checks if the given index is out of range
     * \param index1 reference to the first index number
     * \param index2 reference to the second index number
     * \param dimension1 reference to the first index dimension
     * \param dimension2 reference to the second index dimension
     */
    void check_index( const int& index1, const int& index2, const int& dimension1, const int& dimension2) const;

    /*!
     * \brief Check proper orthogonal matrix
     *
     * This functions checks if the given matrix is a proper orthogonal matrix
     * \param matrix reference to a 3 by 3 matrix
     * \return boolean value
     */
    bool is_rotation_matrix(const Eigen::Matrix3d& matrix) const;

};

//Non-member functions

/*!
 * \brief Convert euler angle to rotation matrix
 *
 * This functions take the euler angle set following the 3-2-1 convention and convert it to rotation matrix R_wb
 * \param euler reference to constant 3 by 1 euler angle set
 * \return The converted rotation matrix R_wb
 */
Eigen::Matrix3d get_R_from_euler(const Eigen::Vector3d& euler);

/*!
 * \brief Convert euler angle to rotation matrix
 *
 * This functions take the euler angle set following the 3-2-1 convention and convert it to rotation matrix R_wb
 * \param roll reference to constant roll angle
 * \param pitch reference to constant pitch angle
 * \param yaw reference to constant yaw angle
 * \return The converted rotation matrix R_wb
 */
Eigen::Matrix3d get_R_from_euler(const double& roll, const double& pitch, const double& yaw);

/*!
 * \brief Convert rotation matrix to quaternion
 *
 * This functions take the rotation matrix and convert it to quaternion(x,y,z,w)
 * \param R 3 by 3 rotation matrix
 * \return 4 by 1 quaternion(x,y,z,w)
 */
Eigen::Vector4d get_quaternion_from_R(const Eigen::Matrix3d& R);

/*!
 * \brief Convert quaternion to rotation matrix
 *
 * This functions take the quaternion(x,y,z,w) and convert it to rotation matrix
 * \param q 4 by 1 quaternion(x,y,z,w)
 * \return 3 by 3 rotation matrix
 */
Eigen::Matrix3d get_R_from_quaternion(const Eigen::Vector4d& q);

/*!
 * \brief Convert rotation matrix to euler angle
 *
 * This functions take the rotation matrix R_wb and converts to euler angle set following the 3-2-1 convention
 * \param R 3 by 3 rotation matrix R_wb
 * \return The converted euler angle set
 */
Eigen::Vector3d get_euler_from_R(const Eigen::Matrix3d& R);

#endif //NEW_SIMULATOR_STATE_H
