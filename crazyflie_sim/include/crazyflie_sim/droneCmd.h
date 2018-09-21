/*
 * droneCmd.h
 *
 *  Created On : 29/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_DRONECMD_H
#define NEW_SIMULATOR_DRONECMD_H

#include <Eigen/Dense>
#include <string>
#include <iostream>

//! DroneCmd class.
/*!
 * DroneCmd class contains the latest command information for a single drone.
 * The command is either the “PosSet” or “AltHold” that are explained from section 2.2.1.
 * Besides the command value, this class has other two private data members
 * to indicate whether a command exist or not and the type of the latest command.
 * Eigen library is used to store and pass vectors and matrices as its Matrix object.
 */
class DroneCmd{

public:

    /****Constructors****/

    //! default consturctor
    DroneCmd() = default;

    //! constructor that take a id value
    explicit DroneCmd(const int& id_value):id(id_value){}

    /****Member functions****/
    // Mutator functions
    DroneCmd& set_cmd_exist(bool value){cmd_exist = value;}                                                 /*!< set the cmd_exit private member */
    DroneCmd& set_cmd(const Eigen::Vector4d& value){cmd = value;}                                           /*!< set the cmd private member */
    DroneCmd& set_cmd(const int& index, const double& value){check_index(index, 4); cmd(index) = value;}    /*!< set the cmd private member */
    DroneCmd& set_mode(const std::string& value){mode = value;}                                             /*!< set the mode private member */

    // Accessor functions
    bool get_cmd_exist() const{return cmd_exist;}                                                           /*!< get the cmd_exit private member */
    const Eigen::Vector4d& get_cmd() const {return cmd;}                                                    /*!< get the cmd private member */
    const double& get_cmd(const int& index) const {check_index(index, 4); return cmd(index);}               /*!< get the cmd private member */
    const std::string& get_mode() const {return mode;}                                                      /*!< get the mode private member */
    const int& get_id() const {return id;}                                                                  /*!< get the id private member */

private:

    int id = 0;                                                                         /*!< object id number */
    bool cmd_exist = false;                                                             /*!< indicator for whether the command exist */
    Eigen::Vector4d cmd = Eigen::Vector4d::Zero();                                      /*!< 4 by 1 vector for command info from either "PosSet" or "AltHold" command */
    std::string mode = "NA";                                                            /*!< command type, either "PosSet" or "AltHold" */

    /*!
     * \brief Check index
     *
     * This functions checks if the given index is out of range
     * \param index reference to index number
     * \param size reference to the index dimension
     */
    void check_index( const int& index, const int& size ) const{

        if (index > (size - 1)){
            std::cerr << "[DroneCmd " << id <<"]: index(" << index << ") out of range!"  << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

};

#endif //NEW_SIMULATOR_DRONECMD_H
