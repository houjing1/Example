/*
 * pid.cpp
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#include "crazyflie_sim/pid.h"
#include "crazyflie_sim/utility.h"

// Static members
double Pid::DEFAULT_INTEGRATION_LIMIT = 5000.0;
double Pid::DEFAULT_OUTPUT_LIMIT = 0.0;

// Update pid controller internal states and produce output
double Pid::update_pid(const double& measure, bool updateError){

    if (updateError){
        error = desired - measure;
    }

    deriv = (error - prevError) / dt;

    integ += error * dt;
    if(iLimit != 0.0){
        integ = clamp(integ, -iLimit, iLimit);
    }

    double output = kp * error + ki * integ + deriv * kd;
    if (outputLimit != 0.0){
        output = clamp(output, -outputLimit, outputLimit);
    }

    prevError = error;

    return output;

}

