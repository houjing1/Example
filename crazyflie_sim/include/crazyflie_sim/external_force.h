//
// Created by houjing1 on 29/06/18.
//

#ifndef NEW_SIMULATOR_EXTERNAL_FORCE_H
#define NEW_SIMULATOR_EXTERNAL_FORCE_H

#include <Eigen/Dense>
#include "crazyflie_sim/state.h"
#include "crazyflie_sim/utility.h"
#include "crazyflie_sim/parameter.h"
#include "parameter.h"
#include <iostream>

Eigen::Vector3d random_force (const State& state){

    static Parameter param;
    static Eigen::Vector3d sampleVec(0,0,0);
    static EigenMultivariateNormal<double, 3> generator(param.acc_mean, param.acc_cov);
    generator.nextSample(sampleVec);

    //std::cout << "----------" << std::endl;
    //std::cout << sampleVec << std::endl;
    return (sampleVec * param.m);

}


Eigen::Vector3d random_walk (const State& state){

    static Eigen::Matrix3d R_fb = Eigen::Matrix3d::Identity();
    static Parameter param;
    Eigen::Vector3d euler;
    Eigen::Vector3d force(3.0e-3,0.0,0.0);

    euler = multivariate_normal<double, 3>(param.angle_mean, param.angle_cov);

    //euler << 0.0, 0.0, 8.0e-5;
    R_fb = get_R_from_euler(euler) * R_fb; // new R_fb

    Eigen::Vector3d global_force = state.get_R() * R_fb.transpose() * force;

    //std::cout << "----------" << std::endl;
    //std::cout << global_force(0) << std::endl;
    //std::cout << euler << std::endl;
    return global_force;


}


#endif //NEW_SIMULATOR_EXTERNAL_FORCE_H
