/*
 * state.cpp
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/state.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// overloaded consturctor that takes in a single id value
State::State(const int& id_value):id(id_value){
    R = get_R_from_euler();

    Eigen::Quaterniond q(R);    // set quaternion from rotation matrix
    quaternion = q;
}

// overloaded constructor. Assuming id_value for drone starts from 1
State::State(const int& id_value, const double& x_init, const double& y_init):id(id_value){

    // ensure id starts from 1
    if (id_value < 1){
        std::cerr << "[State " << id <<"]: invalid id number " << id_value << " (id starts from 1)"  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    pos(0) = x_init;            // set initial x position from argument
    pos(1) = y_init;            // set initial y position from argument

    R = get_R_from_euler();     // set rotation matrix from initialized euler angle

    Eigen::Quaterniond q(R);    // set quaternion from rotation matrix
    quaternion = q;
}

// overloaded constructor. Assuming id_value for drone starts from 1
State::State(const int& id_value, const DroneSimInfo& info):id(id_value){

    // ensure id starts from 1
    if (id_value < 1){
        std::cerr << "[State " << id <<"]: invalid id number " << id_value << " (id starts from 1)"  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    pos(0) = info.get_x_init(id_value - 1);     // set initial x position from DroneSimInfo object
    pos(1) = info.get_y_init(id_value - 1);     // set initial y position from DroneSimInfo object

    R = get_R_from_euler();                     // set rotation matrix from initialized euler angle

    Eigen::Quaterniond q(R);                    // set quaternion from rotation matrix
    quaternion = q;
}

// Check for out of range index
void State::check_index( const int& index, const int& dimension) const{

    if (index > (dimension - 1)){
        std::cerr << "[State " << id <<"]: index(" << index << ") out of range!"  << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Check for out of range index
void State::check_index( const int& index1, const int& index2, const int& dimension1, const int& dimension2) const{

    if (index1 > (dimension1 - 1) || index2 > (dimension2 - 1)){
        std::cerr << "[State " << id <<"]: index(" << index1 << ", " << index2 <<") out of range!" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Check if the matrix is a proper rotation matrix
// Reference: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
bool State::is_rotation_matrix(const Eigen::Matrix3d& matrix) const {
    Eigen::Matrix3d temp = matrix * matrix.transpose();
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    return (identity - temp).norm() < 1e-6;
}

// Calculate euler angles(in radian) from rotation matrix.
// Assuming intrinsic 1-2-3 sequence convention and active rotation convention
// Return a 3 by 1 vector of roll, pitch, yaw
// Reference: http://danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html
Eigen::Vector3d State::get_euler_from_R() const{

    if ( !is_rotation_matrix(R) ){
        //std::cerr << "[State " << id <<"]: Improper rotation matrix!" << std::endl;
        //std::exit(EXIT_FAILURE);
    }

    double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

    bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular){
        yaw = std::atan2(R(1,0), R(0,0));
        pitch = -std::asin(R(2,0));
        roll = std::atan2(R(2,1), R(2,2));
    }

    else{
        std::cerr << "[State " << id <<"]: Singularity occurred" << std::endl;
        yaw = 0.0;
        pitch = -std::asin( static_cast<float>(R(2,0)) );
        if (pitch < 0.0){
            roll = std::atan2(-R(0,1), -R(0,2));
        }
        else{
            roll = std::atan2(R(0,1), R(0,2));
        }
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

// Calculate quaternion from rotation matrix
// Return a 4 by 1 vector of x,y,z,w
Eigen::Vector4d State::get_quaternion_from_R() const{

    Eigen::Quaterniond q(R);
    Eigen::Vector4d quaternion = q.normalized().coeffs();

    return quaternion;
}

// Reset all data member
void State::reset(){

     R = Eigen::Matrix3d::Identity();
     pos = Eigen::Vector3d::Zero();
     vel = Eigen::Vector3d::Zero();
     acc = Eigen::Vector3d::Zero();
     omega = Eigen::Vector3d::Zero();
     euler = Eigen::Vector3d::Zero();
}

// Calculate rotation matrix from euler angles
// Input euler vector should be (roll, pitch, yaw) in radian
// Returned rotation matrix is R_wb

// member function version
Eigen::Matrix3d State::get_R_from_euler() const{

    Eigen::Matrix3d R; // R_bw

    R = Eigen::AngleAxisd(-euler(0), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-euler(2), Eigen::Vector3d::UnitZ());

    return R.transpose(); // R_wb
}

// Non member functions

// non member function version
Eigen::Matrix3d get_R_from_euler(const Eigen::Vector3d& euler){

    Eigen::Matrix3d R; // R_bw

    R = Eigen::AngleAxisd(-euler(0), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-euler(2), Eigen::Vector3d::UnitZ());

    return R.transpose(); // R_wb
}

// non member function version
// Overloaded get_R_from_euler function with 3 double type angle inputs
Eigen::Matrix3d get_R_from_euler(const double& roll, const double& pitch, const double& yaw){

    Eigen::Matrix3d R; // R_bw

    R = Eigen::AngleAxisd(-roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ());

    return R.transpose(); // R_wb
}

// Calculate quaternion from rotation matrix(R_wb)
// Return a 4 by 1 vector of x,y,z,w
Eigen::Vector4d get_quaternion_from_R(const Eigen::Matrix3d& R){

    Eigen::Quaterniond q(R);
    Eigen::Vector4d quaternion = q.normalized().coeffs();

    return quaternion;
}

// Calculate rotation matrix from quaternion(x,y,z,w)
// Return a 3 by 3 rotation matrix
Eigen::Matrix3d get_R_from_quaternion(const Eigen::Vector4d& q_vec){

    Eigen::Quaterniond q(q_vec);
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    return R;
}

// Calculate euler angles(in radian) from rotation matrix.
// Assuming intrinsic 1-2-3 sequence convention and active rotation convention
// Return a 3 by 1 vector of roll, pitch, yaw
// Reference: http://danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html
Eigen::Vector3d get_euler_from_R(const Eigen::Matrix3d& R) {

    double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

    bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular){
        yaw = std::atan2(R(1,0), R(0,0));
        pitch = -std::asin(R(2,0));
        roll = std::atan2(R(2,1), R(2,2));
    }

    else{
        std::cerr << "Singularity occurred" << std::endl;
        yaw = 0.0;
        pitch = -std::asin( static_cast<float>(R(2,0)) );
        if (pitch < 0.0){
            roll = std::atan2(-R(0,1), -R(0,2));
        }
        else{
            roll = std::atan2(R(0,1), R(0,2));
        }
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}