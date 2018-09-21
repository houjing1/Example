//
// Created by houjing1 on 06/08/18.
//

#ifndef NEW_SIMULATOR_POSITIONCONTROLLER_M_H
#define NEW_SIMULATOR_POSITIONCONTROLLER_M_H

#include <Eigen/Dense>
#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/baseController.h"

class PositionController_m : public BaseController {

public:
    /****Constructors****/

    //! constructor function that initializes id, param and pid objects
    explicit PositionController_m(const int& id_value);

    /****Member functions****/
    // Accessor functions

    const int& get_id() const {return id;}                                      /*!< get id number */
    const double& get_desired_thrust() const {return thrust_desired;}           /*!< get desired base thrust */
    const Eigen::Vector3d& get_desired_euler() const {return euler_desired;}    /*!< get desired euler command angle */
    const Eigen::Matrix3d& get_desired_R() const {return R_desired;}            /*!< get desired rotation matrix R_wb */

    // Utility functions

    /*!
     * \brief update overall controller
     *
     * Perform update on the entire controller and return output vector as desired thrust-roll-pitch.
     * This function overrides its base class function.
     *
     * \param setpoint is a Setpoint object that contains the desired position, velocity, acceleration and yaw angle(rad) [desired_pos, desired_vel, desired_acc, desired_yaw]
     * \param state is a State object that contains the measured position, velocity, and current rotation matrix [measured_pos, measured_vel, current_R]
     * \return base class Control object that contains desired thrust(pwm) and desired euler angle(rad): [desired_thrust desired_euler]
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

    int id = 0;                                                                 /*!< object id number */
    static Parameter param;                                                     /*!< Parameter object */
    double dt;                                                                  /*!< timer interval */

    double c_2 = -73820.0;
    double c_1 = 147723.96;
    double c   = 1369.68;

    // XY Position PID
    double kp_xy = 0.4;
    //double kp_xy = 0.6;
    double kd_xy = 0.2;
    double ki_xy = 0.05;
    double i_range_xy = 2.0;

    // Z Position
    double kp_z = 1.25;
    //double kp_z = 0.5;
    double kd_z = 0.5;
    double ki_z = 0.05;
    double i_range_z = 0.15;

    // Helper variables

    double i_error_x = 0;
    double i_error_y = 0;
    double i_error_z = 0;

    // Member data
    double thrust_desired = 0;
    Eigen::Vector3d euler_desired = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_desired = Eigen::Matrix3d::Identity();

};

#endif //NEW_SIMULATOR_POSITIONCONTROLLER_M_H
