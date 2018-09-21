/*
 * utility.h
 *
 *  Created On : 22/06/18
 *      Author : Jingyuan Hou
 *      Email  : jingyuan.hou@mail.utoronto.ca
 */

#ifndef NEW_SIMULATOR_UTILITY_H
#define NEW_SIMULATOR_UTILITY_H

#include <functional>
#include <chrono>
#include <random>
#include <Eigen/Dense>

//! Contain user defined utility functions.


/*!
 * \brief clamping function
 *
 * Bound the input within in a interval
 *
 * \param v reference to a value to be clamped
 * \param lo reference to a lower bound value
 * \param hi reference to a higher bound value
 * \return The reference to the limit value if input is output bound, otherwise return the reference to the input v.
 */
constexpr const double& clamp( const double& v, const double& lo, const double& hi )
{
    if (v > hi){
        return hi;
    }
    else if (v < lo){
        return lo;
    }
    return v;

}

/*!
 * \brief Random number generator
 *
 * Generate a random number from normal distribution
 *
 * \param mean mean value from normal distribution
 * \param std standard deviation from normal distribution
 * \return The random number
 */
extern double normal_distribution_generator(double mean = 0.0, double std = 1.0);

// generate number from multivariate normal distribution
//extern Eigen::MatrixXd multivariate_normal_distribution(Eigen::VectorXd mean = Eigen::VectorXd::Zero(), Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity());

#include <Eigen/Dense>

#include <cmath>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <time.h>
#include <chrono>

/**
    We find the eigen-decomposition of the covariance matrix.
    We create a vector of normal samples scaled by the eigenvalues.
    We rotate the vector by the eigenvectors.
    We add the mean.
*/
template<typename _Scalar, int _size>
class EigenMultivariateNormal
{
    //static_cast<unsigned int>( std::time(0))
    boost::mt19937 rng = boost::mt19937(std::chrono::high_resolution_clock::now().time_since_epoch().count());    // The uniform pseudo-random algorithm
    boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
            randN; // The 0-mean unit-variance normal generator

    Eigen::Matrix<_Scalar,_size,_size> rot;
    Eigen::Matrix<_Scalar,_size,1> scl;

    Eigen::Matrix<_Scalar,_size,1> mean;

public:
    EigenMultivariateNormal(const Eigen::Matrix<_Scalar,_size,1>& meanVec,
                            const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
            : randN(rng,norm)
    {
        setCovar(covarMat);
        setMean(meanVec);
    }

    void setCovar(const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<_Scalar,_size,_size> >
                eigenSolver(covarMat);
        rot = eigenSolver.eigenvectors();
        scl = eigenSolver.eigenvalues();
        for (int ii=0;ii<_size;++ii) {
            scl(ii,0) = sqrt(scl(ii,0));
        }
    }

    void setMean(const Eigen::Matrix<_Scalar,_size,1>& meanVec)
    {
        mean = meanVec;
    }

    void nextSample(Eigen::Matrix<_Scalar,_size,1>& sampleVec)
    {
        for (int ii=0;ii<_size;++ii) {
            sampleVec(ii,0) = randN()*scl(ii,0);
        }
        sampleVec = rot*sampleVec + mean;
    }

};

/*!
 * \brief Multivariate normal distribution number generator
 *
 * Generate a random N dimension vector from normal distribution
 *
 * \param mean N by 1 value from normal distribution
 * \param cov N by N covariance matrix
 * \return The random N by 1 vector
 */
template <typename T, int size>
extern Eigen::Matrix<T, size, 1> multivariate_normal (const Eigen::Matrix<T, size, 1>& mean, const Eigen::Matrix<T, size, size>& cov){
    Eigen::Matrix<T, size, 1> sampleVec;
    EigenMultivariateNormal<T, size> generator(mean, cov);
    generator.nextSample(sampleVec);

    return sampleVec;
}

/*!
 * \brief Compute moving average of data
 *
 * Compute moving average of N number of data with M dimension
 *
 * \param data_matrix is a M by N matrix with each column a M by 1 data
 * \param data is a M by 1 vector data
 */
template <typename T, int dimension, int num_of_data>
void average_data(Eigen::Matrix<T, dimension, num_of_data>& data_matrix, Eigen::Matrix<T, dimension, 1>& data){

    data = (data_matrix.rowwise().sum() + data) / (num_of_data+1);

    for(int i = 0; i < num_of_data-1; i++){
        data_matrix.col(i) = data_matrix.col(i+1);
    }

    data_matrix.col(num_of_data-1) = data;

}


//! Setpoint class for passing desired values
class Setpoint{

public:

    Setpoint() = default;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();

};

//! Control class for passing controller outputs
class Control{

public:

    Control() = default;

    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_variation = Eigen::Vector3d::Zero();
    double thrust = 0.0;

};


#endif //NEW_SIMULATOR_UTILITY_H
