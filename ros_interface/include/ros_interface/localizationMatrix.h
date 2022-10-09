//
// Created by valerio on 01/08/17.
//

#ifndef CARRELLINO_LOCALIZATIONMATRIX_H
#define CARRELLINO_LOCALIZATIONMATRIX_H

#include <Eigen/src/Core/Matrix.h>

namespace localizationMatrix{

    const int STATE_SIZE=5;
    ///< Define some useful types.
    typedef Eigen::Matrix<double,STATE_SIZE,1> StateVec_t;
    typedef Eigen::Matrix<double,STATE_SIZE,STATE_SIZE> StateCovMatrix_t;
}
#endif //CARRELLINO_LOCALIZATIONMATRIX_H
