/*
 * droneSimIfo.h
 *
 *  Created On : 25/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_DRONESIMINFO_H
#define NEW_SIMULATOR_DRONESIMINFO_H

#include <Eigen/Dense>
#include <vector>
#include <string>

//! The DroneSim class.
/*!
 * DroneSimInfo class contains all the controller parameters and drones’ initial position information passed in from ROS parameter server.
 * These parameter are initially written in a yaml file and loaded into the ROS parameter server upon launching the simulation.
 * Information from DroneSimInfo is used to initialize the Quadrotor object
 * and are then used by Quadrotor class to subsequently initialize its contained classes including: State, PositionController,
 * AttitudeController, PowerDistribution, MotorDynamics and QuadrotorDyanmics objects. Eigen library is used to store
 * and pass vectors and matrices as its Matrix object.
 */
class DroneSimInfo{

public:

    /****Constructors****/

    //! Constructor that takes in the number of drones
    explicit DroneSimInfo(const int& size): num_of_drone(size), name(size), x_init(size), y_init(size){ }

    //! Default constructor
    DroneSimInfo():DroneSimInfo(0){}

    /****Member functions****/

    // Accessor functions for pids that return a single Vector4d value
    const Eigen::Vector4d& get_x_pid() const {return x_pid;}                                                        /*!< get x pid parameter */
    const Eigen::Vector4d& get_y_pid() const {return y_pid;}                                                        /*!< get y pid parameter */
    const Eigen::Vector4d& get_z_pid() const {return z_pid;}                                                        /*!< get z pid parameter */
    const Eigen::Vector4d& get_vx_pid() const {return vx_pid;}                                                      /*!< get vx pid parameter */
    const Eigen::Vector4d& get_vy_pid() const {return vy_pid;}                                                      /*!< get vy pid parameter */
    const Eigen::Vector4d& get_vz_pid() const {return vz_pid;}                                                      /*!< get vz pid parameter */
    const Eigen::Vector4d& get_roll_pid() const {return roll_pid;}                                                  /*!< get roll pid parameter */
    const Eigen::Vector4d& get_pitch_pid() const {return pitch_pid;}                                                /*!< get pitch pid parameter */
    const Eigen::Vector4d& get_yaw_pid() const {return yaw_pid;}                                                    /*!< get yaw pid parameter */
    const Eigen::Vector4d& get_p_pid() const {return p_pid;}                                                        /*!< get p pid parameter */
    const Eigen::Vector4d& get_q_pid() const {return q_pid;}                                                        /*!< get q pid parameter */
    const Eigen::Vector4d& get_r_pid() const {return r_pid;}                                                        /*!< get r pid parameter */

    const double& get_x_pid(const int& index) const {check_index(index, 4); return x_pid(index);}                   /*!< get individual component of x pid parameter */
    const double& get_y_pid(const int& index) const {check_index(index, 4); return y_pid(index);}                   /*!< get individual component of y pid parameter */
    const double& get_z_pid(const int& index) const {check_index(index, 4); return z_pid(index);}                   /*!< get individual component of z pid parameter */
    const double& get_vx_pid(const int& index) const {check_index(index, 4); return vx_pid(index);}                 /*!< get individual component of vx pid parameter */
    const double& get_vy_pid(const int& index) const {check_index(index, 4); return vy_pid(index);}                 /*!< get individual component of vy pid parameter */
    const double& get_vz_pid(const int& index) const {check_index(index, 4); return vz_pid(index);}                 /*!< get individual component of vz pid parameter */
    const double& get_roll_pid(const int& index) const {check_index(index, 4); return roll_pid(index);}             /*!< get individual component of roll pid parameter */
    const double& get_pitch_pid(const int& index) const {check_index(index, 4); return pitch_pid(index);}           /*!< get individual component of pitch pid parameter */
    const double& get_yaw_pid(const int& index) const {check_index(index, 4); return yaw_pid(index);}               /*!< get individual component of yaw pid parameter */
    const double& get_p_pid(const int& index) const {check_index(index, 4); return p_pid(index);}                   /*!< get individual component of p pid parameter */
    const double& get_q_pid(const int& index) const {check_index(index, 4); return q_pid(index);}                   /*!< get individual component of q pid parameter */
    const double& get_r_pid(const int& index) const {check_index(index, 4); return r_pid(index);}                   /*!< get individual component of r pid parameter */


    // Other accessor functions

    const int& get_num_of_drone() const{return num_of_drone;}                                                       /*!< get number of drones */

    const double& get_x_init(const int& index) const { check_index(index); return x_init(index); }                  /*!< get individual component of x_init member */
    const double& get_y_init(const int& index) const { check_index(index); return y_init(index); }                  /*!< get individual component of y_init member */
    const std::string& get_name(const int& index) const{ check_index(index); return name[index]; }                  /*!< get individual component of name member */
    const std::vector<std::string>& get_name() const{ return name; }                                                /*!< get name member */
    const std::string& get_controller_type() const{return controller_type;}                                         /*!< get controller_type member */
    const bool& get_noise_flag() const{return include_noise;}                                                       /*!< get include_noise member */
    const bool& get_disturbance_flag() const{return include_disturbance;}                                           /*!< get include_disturbance member */
    const std::string& get_reference_source() const{return reference_source;}                                       /*!< get reference_source member */

    // Mutator functions for pids that take a single Vector4d argument
    DroneSimInfo& set_x_pid(const Eigen::Vector4d& value){x_pid = value; return *this; }                            /*!< set x pid parameter */
    DroneSimInfo& set_y_pid(const Eigen::Vector4d& value){y_pid = value; return *this; }                            /*!< get y pid parameter */
    DroneSimInfo& set_z_pid(const Eigen::Vector4d& value){z_pid = value; return *this; }                            /*!< get z pid parameter */
    DroneSimInfo& set_vx_pid(const Eigen::Vector4d& value){vx_pid = value; return *this; }                          /*!< get vx pid parameter */
    DroneSimInfo& set_vy_pid(const Eigen::Vector4d& value){vy_pid = value; return *this; }                          /*!< get vy pid parameter */
    DroneSimInfo& set_vz_pid(const Eigen::Vector4d& value){vz_pid = value; return *this; }                          /*!< get vz pid parameter */
    DroneSimInfo& set_roll_pid(const Eigen::Vector4d& value){roll_pid = value; return *this; }                      /*!< get roll pid parameter */
    DroneSimInfo& set_pitch_pid(const Eigen::Vector4d& value){pitch_pid = value; return *this; }                    /*!< get pitch pid parameter */
    DroneSimInfo& set_yaw_pid(const Eigen::Vector4d& value){yaw_pid = value; return *this; }                        /*!< get yaw pid parameter */
    DroneSimInfo& set_p_pid(const Eigen::Vector4d& value){p_pid = value; return *this; }                            /*!< get p pid parameter */
    DroneSimInfo& set_q_pid(const Eigen::Vector4d& value){q_pid = value; return *this; }                            /*!< get q pid parameter */
    DroneSimInfo& set_r_pid(const Eigen::Vector4d& value){r_pid = value; return *this; }                            /*!< get r pid parameter */

    // Mutator functions for pids that take 4 double arguments
    DroneSimInfo& set_x_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){x_pid(0) = kp; x_pid(1) = ki; x_pid(2) = kd; x_pid(3) = iLimit; return *this;}                          /*!< set x pid parameter */
    DroneSimInfo& set_y_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){y_pid(0) = kp; y_pid(1) = ki; y_pid(2) = kd; y_pid(3) = iLimit; return *this;}                          /*!< set y pid parameter */
    DroneSimInfo& set_z_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){z_pid(0) = kp; z_pid(1) = ki; z_pid(2) = kd; z_pid(3) = iLimit; return *this;}                          /*!< set z pid parameter */
    DroneSimInfo& set_vx_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){vx_pid(0) = kp; vx_pid(1) = ki; vx_pid(2) = kd; vx_pid(3) = iLimit; return *this;}                     /*!< set vx pid parameter */
    DroneSimInfo& set_vy_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){vy_pid(0) = kp; vy_pid(1) = ki; vy_pid(2) = kd; vy_pid(3) = iLimit; return *this;}                     /*!< set vy pid parameter */
    DroneSimInfo& set_vz_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){vz_pid(0) = kp; vz_pid(1) = ki; vz_pid(2) = kd; vz_pid(3) = iLimit; return *this;}                     /*!< set vz pid parameter */
    DroneSimInfo& set_roll_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){roll_pid(0) = kp; roll_pid(1) = ki; roll_pid(2) = kd; roll_pid(3) = iLimit; return *this;}           /*!< set roll pid parameter */
    DroneSimInfo& set_pitch_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){pitch_pid(0) = kp; pitch_pid(1) = ki; pitch_pid(2) = kd; pitch_pid(3) = iLimit; return *this;}      /*!< set pith pid parameter */
    DroneSimInfo& set_yaw_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){yaw_pid(0) = kp; yaw_pid(1) = ki; yaw_pid(2) = kd; yaw_pid(3) = iLimit; return *this;}                /*!< set yaw pid parameter */
    DroneSimInfo& set_p_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){p_pid(0) = kp; p_pid(1) = ki; p_pid(2) = kd; p_pid(3) = iLimit; return *this;}                          /*!< set p pid parameter */
    DroneSimInfo& set_q_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){q_pid(0) = kp; q_pid(1) = ki; q_pid(2) = kd; q_pid(3) = iLimit; return *this;}                          /*!< set q pid parameter */
    DroneSimInfo& set_r_pid(const double& kp, const double& ki, const double& kd, const double& iLimit){r_pid(0) = kp; r_pid(1) = ki; r_pid(2) = kd; r_pid(3) = iLimit; return *this;}                          /*!< set r pid parameter */

    // Other mutator functions
    void set_x_init(const int& index, const double& value){check_index(index); x_init(index) = value;}              /*!< set x_init member */
    void set_y_init(const int& index, const double& value){check_index(index); y_init(index) = value;}              /*!< set y_init member */
    void set_name(const int& index, const std::string& name_value){check_index(index); name[index] = name_value;}   /*!< set name member */
    void set_controller_type(const std::string& type_name ){controller_type = type_name;}                           /*!< set controller_type member */
    void set_noise_flag(const bool& flag){include_noise = flag;}                                                    /*!< set flag for include measurement noise */
    void set_disturbance_flag(const bool& flag){include_disturbance = flag;}                                        /*!< set flag for include disturbance forces */
    void set_reference_source(const std::string& source_name){reference_source = source_name;}                     /*!< set reference_source member */

    // Utility functions

    /*!
     * \brief Resize
     *
     * Resize the number of drone as well as the resize of array of names and x, y initial positions.
     */
    void resize(const int& size);


private:

    int num_of_drone = 0;                                   /*!< number of drones */
    std::vector<std::string> name;                          /*!< list of drone names */
    std::string controller_type = "pid";                    /*!< controller type */
    std::string reference_source = "true_state";            /*!< source of controler reference signal */
    bool include_noise = false;                             /*!< flag for including measurement noise */
    bool include_disturbance = false;                       /*!< flag for including disturbance forces */

    Eigen::VectorXd x_init;                                 /*!< list of x initial position of each drone */
    Eigen::VectorXd y_init;                                 /*!< list of y initial position of each drone */

    Eigen::Vector4d x_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for x pid */
    Eigen::Vector4d y_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for y pid */
    Eigen::Vector4d z_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for z pid */
    Eigen::Vector4d vx_pid = Eigen::Vector4d::Zero();       /*!< list of parameters for vx pid */
    Eigen::Vector4d vy_pid = Eigen::Vector4d::Zero();       /*!< list of parameters for vy pid */
    Eigen::Vector4d vz_pid = Eigen::Vector4d::Zero();       /*!< list of parameters for vz pid */
    Eigen::Vector4d roll_pid = Eigen::Vector4d::Zero();     /*!< list of parameters for roll pid */
    Eigen::Vector4d pitch_pid = Eigen::Vector4d::Zero();    /*!< list of parameters for pitch pid */
    Eigen::Vector4d yaw_pid = Eigen::Vector4d::Zero();      /*!< list of parameters for yaw pid */
    Eigen::Vector4d p_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for p pid */
    Eigen::Vector4d q_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for q pid */
    Eigen::Vector4d r_pid = Eigen::Vector4d::Zero();        /*!< list of parameters for r pid */

    // Assistant functions

    /*!
     * \brief Check index
     *
     * This functions checks if the given index is out of range base on the number of drones
     * \param index reference to index number
     */
    void check_index( const int& index ) const;

    /*!
     * \brief Check index
     *
     * This functions checks if the given index is out of range
     * \param index reference to index number
     * \param size reference to the index dimension
     */
    void check_index( const int& index, const int& size ) const;

};

#endif //NEW_SIMULATOR_DRONESIMINFO_H
