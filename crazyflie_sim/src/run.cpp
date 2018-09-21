/*
 * run.cpp
 *
 *  Created On : 05/07/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "ros/ros.h"
#include "crazyflie_sim/droneSim.h"
#include <iostream>
#include <string>

int main(int argc, char **argv) {

    ros::init(argc, argv, "Simulator");
    ros::NodeHandle nh;
    //ros::NodeHandle private_nh("~");

    std::string path("vehicles/num");
    std::string key;
    if( nh.searchParam(path, key) ){

        int num_of_drone;
        nh.getParam(key, num_of_drone);
        std::cout<<std::endl;
        ROS_INFO_STREAM("[Simulator]: Number of vehicle found: " << num_of_drone << " from " << key);

        DroneSim drone_sim(num_of_drone, nh);
        drone_sim.run();
    }

    else {
        ROS_FATAL_STREAM("[Simulator]: Number of vehicle not found from " << path);
        exit(1);
    }

    return 0;
}