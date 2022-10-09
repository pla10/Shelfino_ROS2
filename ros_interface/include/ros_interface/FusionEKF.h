/*!****************************************************************************
 *    \file    FusionEKF.hpp
 *    \brief   Class that interface with kalman filter and extend it the nonlinear case of the TRONIC walker
 *    \version 1.0
 *    \date    2017
 *****************************************************************************/
/// @file   Algorithm.hpp
/// @brief  Class that interface with kalman filter and extend it to nonlinear case
/// @author Valerio Magnago
///

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <measurement_package.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <mutex>
#include <linear_kalman_filter.h>
#include <walkerKinematic.h>
#include <localizationMatrix.h>
#include <json.hpp>


class FusionEKF {
public:

    /**
    * Constructor.
    */
    FusionEKF();

    /**
    * Destructor.
    */
    virtual ~FusionEKF();

    /**
    * Run the whole flow of the Kalman Filter from here.
    */

    void InitializeFilter(const  localizationMatrix::StateVec_t& x_, const localizationMatrix::StateCovMatrix_t& P_);


    /**
    * Run the whole flow of the Kalman Filter from here.
    */

    bool InitializeFilterJSON(const nlohmann::json &j);


    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);


    double normalizeAngle(double theta);

    int getDataTime(){
        return previous_timestamp_;
    }

    void getLastFilterInformation(int &time,  localizationMatrix::StateVec_t &state, localizationMatrix::StateCovMatrix_t &covariance);

private:
    std::mutex process_measure_mtx;  // Mutex is needed for regulate the ProcessMeasurement.
    bool is_initialized_;   // check whether the tracking toolbox was initiallized or not (first measurement)


    int previous_timestamp_ = 0;     // time of the measure of the last kalman update
    localizationMatrix::StateVec_t vehicle_s;         // state vector of the vehicle
    localizationMatrix::StateCovMatrix_t vehicle_P;   // covariance matrix of the state matrix
    Eigen::Matrix<double,2,2> Q_pred;                  // Prediction covariance of the unmodeled quantities i.e. acceleration and angular acceleration


    walkerKinematic kinematic;   // equation for the walker kinematic and data

    /**
    * Linear kalman filter equation.
    */
    LinearKalmanFilterEq<5> lin_kf;

    /**
     * Propagate the state s and the covariance matrix P ahead. The model consider v_dot and theda_dot_dot with mean 0 and
     * with a variance that is tunable.
     *
     * @param s input initial state, output propagated state
     * @param P  input state covariance, output propagated state covariance
     * @param dt time to propagate ahed
     *
     */
    void nonlinearStatePrediction(localizationMatrix::StateVec_t& s, localizationMatrix::StateCovMatrix_t& P, const double dt);

};

#endif /* FusionEKF_H_ */
