//
// Created by houjing1 on 06/08/18.
//

#ifndef NEW_SIMULATOR_ATTITUDECONTROLLER_M_H
#define NEW_SIMULATOR_ATTITUDECONTROLLER_M_H

#include <Eigen/Dense>
#include "crazyflie_sim/parameter.h"
#include "crazyflie_sim/baseController.h"

class AttitudeController_m : public BaseController {

public:
    /****Constructors****/

    //! constructor function that initializes id, param and pid objects
    explicit AttitudeController_m(const int& id_value);

    /****Member functions****/
    // Accessor functions

    const int& get_id() const {return id;}                                              /*!< get id number */
    const Eigen::Vector3d& get_desired_motor_var() const {return motor_var_desired;}    /*!< get desired motor input variation */

    // Utility functions

    /*!
     * \brief update overall controller
     *
     * Perform update on the entire controller and return output vector as desired motor input variation.
     * This function overrides its base class function.
     *
     * \param setpoint is a Setpoint obejct that contains desired euler angle(rad), angular velocity(rad/s) [desired_rpy(rad), desired_pqr(rad/s)]
     * \param state is a State object that contains the measured angular velocity(rad/s) and current rotation matrix(Rwb) [measured_pqr(rad/s), current_R]
     * \return base class Control object that contains desired motor input variation(pwm): [desired_motor_var]
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
    double dt;                                                                  /*!< time interval */
    bool flag = false;                                                          /*!< flag to initialize derivative term */

    // Attitude
    double kR_xy = 70000;
    double kw_xy = 20000;
    double ki_m_xy = 0.0;
    double i_range_m_xy = 1.0;

    // Yaw
    double kR_z = 60000;
    double kw_z = 12000;
    double ki_m_z = 500;
    double i_range_m_z = 1500;

    // roll and pitch angular velocity
    double kd_omega_rp = 200;
    //double kd_omega_rp = 0;

    // Helper variables
    double prev_omega_roll;
    double prev_omega_pitch;
    double prev_setpoint_omega_roll;
    double prev_setpoint_omega_pitch;

    double i_error_m_x = 0;
    double i_error_m_y = 0;
    double i_error_m_z = 0;

    // Member data
    Eigen::Vector3d motor_var_desired = Eigen::Vector3d::Zero();
};


#endif //NEW_SIMULATOR_ATTITUDECONTROLLER_M_H
