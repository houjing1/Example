/*
 * pid.h
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_PID_H
#define NEW_SIMULATOR_PID_H

#include <string>

//! The Pid class.
/*!
 * Pid class provides the functionality to update a single pid controller when reference and measured signals are provided.
 * It also contains the pid parameters such as pid gains, output limits, integration limits,
 * time step size and etc that will be used when updating a pid controller.
 */
class Pid {

public:

    /****Constructors****/

    //! Default constructor
    Pid() = default;

    /*!
     * \brief Alternative constructor
     *
     * \param id_value The id number for this pid object
     * \param name_value The name of this pid object
     */
    Pid(const int& id_value, const std::string& name_value): id(id_value), name(name_value) { }

    /****Member functions****/

    // Mutator functions
    Pid& set_integral_limit(const double& limit_value) {iLimit = limit_value; return *this;}         /*!< Mutator for iLimit private member */
    Pid& set_output_limit(const double& limit_value) {outputLimit= limit_value; return *this;}       /*!< Mutator for outputLimit private member */
    Pid& set_error(const double& error_value) {error = error_value; return *this;}                   /*!< Mutator for error private member */
    Pid& set_desired(const double& desired_value) {desired = desired_value; return *this;}           /*!< Mutator for desired private member */

    Pid& set_kp(const double& kp_value) {kp = kp_value; return *this;}                               /*!< Mutator for kp private member */
    Pid& set_ki(const double& ki_value) {ki = ki_value; return *this;}                               /*!< Mutator for ki private member */
    Pid& set_kd(const double& kd_value) {kd = kd_value; return *this;}                               /*!< Mutator for kd private member */
    Pid& set_dt(const double& dt_value) {dt = dt_value; return *this;}                               /*!< Mutator for dt private member */

    static void set_DEFAULT_INTEGRATION_LIMIT(const double& v){DEFAULT_INTEGRATION_LIMIT = v;}       /*!< Mutator for DEFAULT_INTEGRATION_LIMIT static private member */
    static void set_DEFAULT_OUTPUT_LIMIT(const double& v){DEFAULT_OUTPUT_LIMIT = v;}                 /*!< Mutator for DEFAULT_OUTPUT_LIMIT static private member */

    Pid& set_id(const int& id_value){ id = id_value; return *this;}                                  /*!< Mutator for id private member */
    Pid& set_name(const std::string& name_value){ name = name_value; return *this;}                  /*!< Mutator for name private member */

    // Accessor functions
    const double& get_integral_limit() const {return iLimit;}                                        /*!< Accessor for iLimit private member */
    const double& get_output_limit() const {return outputLimit;}                                     /*!< Accessor for outputLimit private member */
    const double& get_error() const {return error;}                                                  /*!< Accessor for error private member */
    const double& get_desired() const {return desired;}                                              /*!< Accessor for desired private member */

    const double& get_kp() const {return kp;}                                                        /*!< Accessor for kp private member */
    const double& get_ki() const {return ki;}                                                        /*!< Accessor for ki private member */
    const double& get_kd() const {return kd;}                                                        /*!< Accessor for kd private member */
    const double& get_dt() const {return dt;}                                                        /*!< Accessor for dt private member */

    static const double& get_DEFAULT_INTEGRATION_LIMIT(){return DEFAULT_INTEGRATION_LIMIT;}          /*!< Accessor for dt private member */
    static const double& get_DEFAULT_OUTPUT_LIMIT(){return DEFAULT_OUTPUT_LIMIT;}                    /*!< Accessor for dt private member */

    const int& get_id() const {return id;}                                                           /*!< Accessor for id private member */
    const std::string& get_name() const {return name;}                                               /*!< Accessor for name private member */


    // Utility functions

    /*!
     * \brief The main function to update the pid object.
     *
     * This function uses the desired and measured signals to compute the output signal from this pid object.
     * \param measure The measured signal
     * \param updateError A boolean to indicate whether or not to perform error calculation
     * \return The output signal from the pid object
     */
    double update_pid(const double& measure, bool updateError);

    /*!
     * \brief Reset the pid object
     *
     * This function resets the internal states of this pid object.
     */
    void reset_pid(){
        error           = 0.0;
        prevError       = 0.0;
        integ           = 0.0;
        deriv           = 0.0;
    }

    /*!
     * \brief Reset the pid limit parameters
     *
     * This function resets both integration and output limits to default values.
     */
    void reset_limit(){
        iLimit          = DEFAULT_INTEGRATION_LIMIT;
        outputLimit     = DEFAULT_OUTPUT_LIMIT;
    }

    /*!
     * \brief Check if the Pid object is still active
     *
     * This function checks if the Pid object is still active by comparing its gains with a small threshold value
     * \return A boolan value that indicate whether or not the Pid is still active
     */
    bool is_active(){ return ( !(kp < 0.0001 && ki < 0.0001 && kd < 0.001) ); }

private:

    std::string name;                                           /*!< name variable */
    int id              = 0;                                    /*!< id number */
    double error        = 0.0;                                  /*!< current error */
    double prevError    = 0.0;                                  /*!< previous error */
    double integ        = 0.0;                                  /*!< integration state */
    double deriv        = 0.0;                                  /*!< derivative state */
    double desired      = 0.0;                                  /*!< desired signal */
    double kp           = 0.0;                                  /*!< kp coefficient */
    double ki           = 0.0;                                  /*!< ki coefficient */
    double kd           = 0.0;                                  /*!< kd coefficient */
    double iLimit       = DEFAULT_INTEGRATION_LIMIT;            /*!< integration limit */
    double outputLimit  = DEFAULT_OUTPUT_LIMIT;                 /*!< output limit */
    double dt           = 0.0;                                  /*!< time step */

    static double DEFAULT_INTEGRATION_LIMIT;                    /*!< default integration limit */
    static double DEFAULT_OUTPUT_LIMIT;                         /*!< default output limit */


};


#endif //NEW_SIMULATOR_PID_H
