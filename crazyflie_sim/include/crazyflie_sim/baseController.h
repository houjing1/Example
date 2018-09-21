/*
 * baseController.h
 *
 *  Created On : 23/07/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_BASECONTROLLER_H
#define NEW_SIMULATOR_BASECONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include "crazyflie_sim/utility.h"
#include "crazyflie_sim/state.h"

//! The BaseController class.
/*!
 * The BaseController class is a abstract class that provide a common interface for other user implemented controllers.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object.
 */
class BaseController{

public:
    /** constructor & destructor **/

    //! Default constructor
    BaseController() = default;

    //! Default destructor
    virtual ~BaseController() = default;

    /** Accessor functions **/

    /*!
     * \brief retrieve controller output
     *
     * Get the output private data member. Other derived classes will have access to this member function
     * \return a Control object that contains the outputs from controller
     */
    const Control& get_output() const {return output;}

    /** Utility functions **/

    /*!
     * \brief perform overall update
     *
     * Perform overall update on the controller and return output vector.
     * All derived classes have to override this function to perform their unique update
     *
     * \param setpoint is a Setpoint object to controller that contains all desired values
     * \param state is a State object to controller that contains measured values
     * \return a Control object that contains controller outputs
     */
    virtual const Control& update_overall(const Setpoint& setpoint, const State& state) = 0;

    /*!
     * \brief rest controller
     *
     * Reset controller internal states.
     * All derived classes have to override this function to perform their unique reset option.
     */
    virtual void reset_controller() = 0;

protected:
    // all derived classes will have access to these members
    Control output;                                           /*!< output from controller as a Control object */

};

#endif //NEW_SIMULATOR_BASECONTROLLER_H
