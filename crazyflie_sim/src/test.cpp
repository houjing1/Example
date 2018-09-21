#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <memory>
#include "crazyflie_sim/pid.h"
#include "crazyflie_sim/utility.h"
/*
#include "Parameter.h"
#include "State.h"
#include "DroneSimInfo.h"
#include "PositionController.h"
#include "AttitudeController.h"
#include "PowerDistribution.h"
#include "MotorDynamics.h"
#include "QuadrotorDynamics.h"
#include "Quadrotor.h"
*/

#include "ros/ros.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
using namespace Eigen;
using namespace std;

const double pi = boost::math::constants::pi<double>();

int main(int argc, char **argv) {

ros::init(argc, argv, "pilot");

// test clamp
    double value = -56.0;
    double limit = 50.0;

    value = clamp(value, -limit, limit);
    cout << "value =" << value << endl;


// test Pid
    Pid pid1(0, "position");
    Pid pid2(10, "position");

    pid1.set_kd(20.0);
    const double &kd_cvalue = pid1.get_kd();
    cout << "pid1: kd = " << kd_cvalue << endl; // should be 20

    double kd_value = pid1.get_kd();
    kd_value = 100;
    cout << "pid1: kd = " << kd_value << endl; // should be 100
    cout << "pid1: kd = " << pid1.get_kd() << endl; // should be 20

    // should both be 5000
    cout << "INTEGRATION_LIMIT =" << pid1.get_DEFAULT_INTEGRATION_LIMIT() << endl;
    cout << "INTEGRATION_LIMIT =" << pid2.get_DEFAULT_INTEGRATION_LIMIT() << endl;
    // should both be 5000
    cout << "integral_limit =" << pid1.get_integral_limit() << endl;
    cout << "integral_limit =" << pid2.get_integral_limit() << endl;

    pid1.set_DEFAULT_INTEGRATION_LIMIT(4000);
    // should both be 4000
    cout << "INTEGRATION_LIMIT =" << pid1.get_DEFAULT_INTEGRATION_LIMIT() << endl;
    cout << "INTEGRATION_LIMIT =" << pid2.get_DEFAULT_INTEGRATION_LIMIT() << endl;
    // should both be 5000
    cout << "integral_limit =" << pid1.get_integral_limit() << endl;
    cout << "integral_limit =" << pid2.get_integral_limit() << endl;

    pid1.reset_limit();
    // pid1 should be 4000, pid1 should be 5000
    cout << "integral_limit =" << pid1.get_integral_limit() << endl;
    cout << "integral_limit =" << pid2.get_integral_limit() << endl;

    // should be no 1 0
    cout << "active? " << pid1.is_active() << " " << pid2.is_active() << endl;


// test utility    
    Eigen::Vector3d mean(0,0,0);
    Eigen::Vector3d sampleVec(0,0,0);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

    EigenMultivariateNormal<double, 3> generator(mean, covariance);

    Eigen::Vector3d sum(0,0,0);
    int len = 100000;
    for(int i = 0; i < len; i++){
        generator.nextSample(sampleVec);
        //std::cout << "random vector: " << sampleVec << std::endl;
        sum += sampleVec;
    }

    std::cout << "mean: " << sum(2)/len << std::endl;




return 0;


}
