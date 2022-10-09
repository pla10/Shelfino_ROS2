/*!****************************************************************************
 *    \file    kalman_filter.hpp
 *    \brief   Class that implement a basic kalman filter (code taken from Udacity) it is a template on the dimention of the state
 *    \version 1.0
 *    \date    2017
 *****************************************************************************/
/// @file   Algorithm.hpp
/// @brief  Class that implement a basic kalman filter (code taken from Udacity)
/// @author Valerio Magnago
///

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


#include <Eigen/Dense>
#include <Eigen/Core>
#include <measurement_package.h>

#include <iostream>

template<int state_dimension=5>   // template on state dimension
class LinearKalmanFilterEq {
public:
	typedef Eigen::Matrix<double,state_dimension,state_dimension> StateCovMatrix_t;
	typedef Eigen::Matrix<double,state_dimension,1> StateVec_t;

	/**
	 * Constructor
	 */
	LinearKalmanFilterEq(){};  // Nothing to inizialize


	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model. x_ = F_ * x_ +w_k
	 * @param x_ is the state of the system, that will be update
	 * @param P_ is the covariance of the state that will be update
	 * @param F is the state transition matrix (keeped constant)
	 * @param Q is the process covariance matrix  (keeped constant)
	 */
	void Predict(StateVec_t& x_,StateCovMatrix_t& P_, const StateCovMatrix_t& F, const StateCovMatrix_t& Q){
        // predict next state using Kalman filter.
        x_ = F * x_;
        Eigen::MatrixXd Ft = F.transpose();
        P_ = F * P_ * Ft + Q;
    }

	/**
	 * Updates the state by using standard linear Kalman Filter update equations
	 * @param x_ is the state of the system, that will be update
	 * @param P_ is the covariance of the state that will be update
	 * @param z The measurement at k+1
	 * @param H Is the measurement matrix
	 * @param R is the measurement covariance matrix
	 */
    template<int output_dimension>   // template on the output dimension
    void Update(StateVec_t& x_, StateCovMatrix_t& P_,
                const Eigen::Matrix<double,output_dimension,1>& z,
                const Eigen::Matrix<double,output_dimension,state_dimension>& H,
                const Eigen::Matrix<double,output_dimension,output_dimension>& R)
    {
        //5x1,5x5,2x1,2x5,2x2
        /**
          * update the state by using Kalman Filter equations
        */
        //std::cout<<"Update"<<std::endl;
        const Eigen::Matrix<double,output_dimension,1>                 y = z - H * x_;
                                                                       // [2x1] = [2x1] - [2x5][5x1]

        const Eigen::Matrix<double,state_dimension,output_dimension>  Ht = H.transpose();

        const Eigen::Matrix<double,output_dimension,output_dimension>  S = H * P_ * Ht + R;

        const Eigen::Matrix<double,output_dimension,output_dimension> Si = S.inverse();

        const Eigen::Matrix<double,state_dimension,output_dimension>   K =  P_ * Ht * Si;

        const Eigen::Matrix<double,state_dimension,state_dimension>   I = Eigen::MatrixXd::Identity(state_dimension, state_dimension);


        x_ = x_ + (K * y);  //new state [5x1] = [5x1] + [5x2][2x1]
        P_ = (I - K * H) * P_; //new covariance
    }

    void UpdateSpeed(StateVec_t& x_, StateCovMatrix_t& P_,
                const Eigen::Matrix<double,2,1>& z,                
                const Eigen::Matrix<double,2,2>& R)
    {
        /**
          * update the state by using Kalman Filter equations
        */
        //std::cout<<"Update"<<std::endl;
        Eigen::Matrix<double,2,1>                 y;
        y(0) = z(0) - x_(3);
        y(1) = z(1) - x_(4);        

        const Eigen::Matrix<double,2,2>  S = P_.bottomRows(2).rightCols(2) + R;

        const Eigen::Matrix<double,2,2>  Si = S.inverse();

        const Eigen::Matrix<double,state_dimension,2>   K =  P_.rightCols(2) * Si;

        const Eigen::Matrix<double,state_dimension,state_dimension>    I = Eigen::MatrixXd::Identity(state_dimension, state_dimension);
        Eigen::Matrix<double,state_dimension,state_dimension>    E = Eigen::MatrixXd::Zero(state_dimension, state_dimension);
        E.rightCols(2) = K;

        x_ = x_ + (K * y);  //new state
        P_ = (I - E) * P_; //new covariance
    }



	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param x_ is the state of the system, that will be update
	 * @param P_ is the covariance of the state that will be update
	 * @param y Is the difference between the the measurement at k+1 and the predicted measurement (z - z_pred)
	 * @param H Is the measurement matrix
	 * @param R is the measurement covariance matrix
	 */
    template<int output_dimension>   // template on the output dimension
	void UpdateEKF(StateVec_t& x_,StateCovMatrix_t& P_,
                   const Eigen::Matrix<double,output_dimension,1>& y,
                   const Eigen::Matrix<double,output_dimension,state_dimension>& H,
                   const Eigen::Matrix<double,output_dimension,output_dimension>& R)
    {
        /**
          * update the state by using Extended Kalman Filter equations
        */
        const Eigen::Matrix<double,state_dimension,output_dimension>  Ht = H.transpose();   // invert the state measure matrix
        const Eigen::Matrix<double,output_dimension,output_dimension> S = H * P_ * Ht + R;  // compute the measure noise
        const Eigen::Matrix<double,output_dimension,output_dimension> Si = S.inverse();
        const Eigen::Matrix<double,state_dimension,output_dimension>    K = P_ * Ht * Si;    // compute kalman gain

        // new estimate.
        x_ = x_ + (K * y);

        const Eigen::Matrix<double,state_dimension,state_dimension> I = Eigen::MatrixXd::Identity(state_dimension, state_dimension); // Identity matrix
        P_ = (I - K * H) * P_;  // propagate covariance
    }

};


#endif /* KALMAN_FILTER_H_ */
