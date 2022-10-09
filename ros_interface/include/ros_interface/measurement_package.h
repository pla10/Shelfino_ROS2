/*!****************************************************************************
 *    \file    measurement_package.hpp
 *    \brief   Class for manage the different possible type of sensor readings
 *    \version 1.0
 *    \date    2017
 *****************************************************************************/
/// @file   measurement_package.hpp
/// @brief  class for managing the different type of data.
/// @author Valerio Magnago
///

#ifndef CARRELLINO_MEASUREMENT_PACKAGE_H
#define CARRELLINO_MEASUREMENT_PACKAGE_H

#include <Eigen/Dense>

class MeasurementPackage {
public:
    int timestamp_;

    enum SensorType{
        ENCODER,
        QR,
        CORRUPT
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

#endif //CARRELLINO_MEASUREMENT_PACKAGE_H
