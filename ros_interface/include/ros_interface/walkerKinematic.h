//
// Created by valerio on 19/07/17.
//

#ifndef CARRELLINO_WALKERKINEMATIC_H
#define CARRELLINO_WALKERKINEMATIC_H

#include <iostream>
#include <Eigen/Dense>
//#include <walker_parameters_provider.hpp>

/// Check if M_PI is defined
#ifndef M_PI
#define M_PI           3.14159265358979323846  /* pi */
#endif

class walkerKinematic {
private:
    double rwL_circ; // circumference left  wheel
    double rwR_circ; // circumference left  wheel
    double rearTrack;     // wheels distance
    double wheelRadius;
    unsigned int encoder_ppr; // encoders tick per revolution
    Eigen::MatrixXd H;       // matrix from speeds to encoders tick
    Eigen::MatrixXd cov_encoders_speed;       // matrix of the covariance of the speed of the encoders readings
    Eigen::MatrixXd cov_param;  // covariance of the walker parameters  in the following order (rwL_circ,rwR_circ,rearTrack)

    double encoWheelReduction;
    const double rad_s2rpm = 60.0/(2.0*M_PI);   // convert rpm to rad/s
public:
    walkerKinematic(){
        
        // GOOD VALUES FOR ODOM + SCAN VIEW 
        encoWheelReduction = 36.; //WalkerParametersProvider::getInstance().getParameters().rearGearRatio_p;
        rearTrack   = 0.393; //WalkerParametersProvider::getInstance().getParameters().rearWidth_p;
        wheelRadius = 0.1142; //WalkerParametersProvider::getInstance().getParameters().wheelRadius_p;
        rwR_circ    = wheelRadius*2.0*M_PI; //WalkerParametersProvider::getInstance().getParameters().wheelRadius_p*2.0*M_PI; // [m] mesured left wheel circunference
        rwL_circ    = 0.99935*wheelRadius*2.0*M_PI; //WalkerParametersProvider::getInstance().getParameters().wheelRadius_p*2.0*M_PI; // [m mesured right wheel circunference
        encoder_ppr = 2048*encoWheelReduction*4; // WalkerParametersProvider::getInstance().getParameters().encoderPPR_p * encoWheelReduction * 4;  // Encoder tick * 4 *10 that are the reduction


        // Compute matrix that convert speeds in tick
        Updatespeeds2DeltaTick();

        cov_encoders_speed = Eigen::MatrixXd::Zero(2,2);
        cov_encoders_speed(0,0) = 1;       // 1 tick on 0.004 acquisition
        cov_encoders_speed(1,1) = cov_encoders_speed(0,0);   // 1 tick on 0.004 acquisition

        cov_param = Eigen::MatrixXd::Zero(3,3);

        // For the moment are all unified at 1 cm
        cov_param(0,0) = 0.005*0.005*4*M_PI*M_PI;      // measure error in m^2   left radius
        cov_param(1,1) = 0.005*0.005*4*M_PI*M_PI;      // measure error in m^2   right radius
        cov_param(2,2) = 0.005*0.005;      // measure error in m^2   wheels distance
    }

    Eigen::MatrixXd& getOutputMatrix(){ return H; }

    /* Unuseful function at the moment
    void tickspeed2walkerSpeed(const  Eigen::Matrix<double,2,1> tick_speed,  Eigen::Matrix<double,2,1>& walkerSpeed,  Eigen::Matrix<double,2,2>& covariance){
        const double& left_speed  =  tick_speed(0);
        const double& right_speed =  tick_speed(1);
        const double r_c =  rwR_circ/encoder_ppr;  // coeff of right wheel
        const double l_c =  rwL_circ/encoder_ppr;  // coeff of left wheel

        walkerSpeed(0) =  (r_c*right_speed+l_c*left_speed)/2;
        walkerSpeed(1) =  (r_c*right_speed-l_c*left_speed)/rearTrack;

        // Compute the covariance

        // Compute the static part of the encoder readings covariance static part (derivative of the equation respect to the tick_speed)
        Eigen::MatrixXd Q_encod_static(2,2);
        Eigen::MatrixXd J_const(2,2);
        J_const(0,0) = 0.5 * l_c;               // diff(walkerSpeed(0), left_speed)
        J_const(0,1) = 0.5 * r_c;               // diff(walkerSpeed(0), right_speed)
        J_const(1,0) = - l_c / rearTrack;       // diff(walkerSpeed(1), left_speed)
        J_const(1,1) =   r_c / rearTrack;       // diff(walkerSpeed(1), right_speed)

        Eigen::MatrixXd J_var(2,3);
        J_var(0,0) = 0.5 * left_speed  / encoder_ppr;               // diff(walkerSpeed(0), rwL_circ)
        J_var(0,1) = 0.5 * right_speed / encoder_ppr;              // diff(walkerSpeed(0), rwR_circ)
        J_var(0,2) = 0;                                            // diff(walkerSpeed(0), rearTrack)
        J_var(1,0) = - left_speed  / (encoder_ppr * rearTrack);     // diff(walkerSpeed(1), rwL_circ)
        J_var(1,1) =   right_speed / (encoder_ppr * rearTrack);    // diff(walkerSpeed(1), rwR_circ)
        J_var(1,2) = - walkerSpeed(1)/rearTrack;                   // diff(walkerSpeed(1), rearTrack)


        const Eigen::MatrixXd J_const_t = J_const.transpose();
        const Eigen::MatrixXd J_var_t   = J_var.transpose();

        // Propagate the uncertanty of encoders and cov_parameters
        covariance =    J_const * cov_encoders_speed * J_const_t + J_var * cov_param * J_var_t;

    }
    */

    void motorRPM2walkerSpeed(const  Eigen::Matrix<double,2,1> encoder_speed,  Eigen::Matrix<double,2,1>& walkerSpeed,  Eigen::Matrix<double,2,2>& covariance){
        // Copmute the RPM of the wheels

        const double& left_speed  =   encoder_speed(0)/encoWheelReduction;
        const double& right_speed =  -encoder_speed(1)/encoWheelReduction;   // the - sign is because when the omega right is negative the walker is going forward

        //
        const double r_c =  rwR_circ/60.0;  //2.0*(M_PI/60.0)*(0.305/2.0);  //rwR_circ/rad_s2rpm;  // coeff of right wheel
        const double l_c =  rwL_circ/60.0;  //2.0*(M_PI/60.0)*(0.305/2.0);  //rwL_circ/rad_s2rpm;  // coeff of left wheel


        walkerSpeed(0) =  (r_c*right_speed+l_c*left_speed)/2.0;    // v
        walkerSpeed(1) =  (r_c*right_speed-l_c*left_speed)/rearTrack;  // omega

        // Compute the covariance

        // Compute the static part of the encoder readings covariance static part (derivative of the equation respect to the tick_speed)
        Eigen::MatrixXd Q_encod_static(2,2);
        //Eigen::MatrixXd J_const(2,2);
        //J_const(0,0) = 0.5 * l_c;               // diff(walkerSpeed(0), left_speed)
        //J_const(0,1) = 0.5 * r_c;               // diff(walkerSpeed(0), right_speed)
        //J_const(1,0) = - l_c / rearTrack;       // diff(walkerSpeed(1), left_speed)
        //J_const(1,1) =   r_c / rearTrack;       // diff(walkerSpeed(1), right_speed)

        Eigen::MatrixXd J_var(2,3);
        J_var(0,0) = 0.5 * left_speed  / rad_s2rpm;               // diff(walkerSpeed(0), rwL_circ)
        J_var(0,1) = 0.5 * right_speed / rad_s2rpm;              // diff(walkerSpeed(0), rwR_circ)
        J_var(0,2) = 0;                                            // diff(walkerSpeed(0), rearTrack)
        J_var(1,0) = - left_speed  / (rad_s2rpm * rearTrack);     // diff(walkerSpeed(1), rwL_circ)
        J_var(1,1) =   right_speed / (rad_s2rpm * rearTrack);    // diff(walkerSpeed(1), rwR_circ)
        J_var(1,2) = - walkerSpeed(1)/rearTrack;                   // diff(walkerSpeed(1), rearTrack)


        //const Eigen::MatrixXd J_const_t = J_const.transpose();
        const Eigen::MatrixXd J_var_t   = J_var.transpose();

        // Propagate the uncertanty of encoders and cov_parameters
        //covariance =    J_const * cov_encoders_speed * J_const_t + J_var * cov_param * J_var_t;
        covariance =    J_var * cov_param * J_var_t;

    }

    void motorSpeed2walkerSpeed(const  Eigen::Matrix<double,2,1> encoder_speed,  Eigen::Matrix<double,2,1>& walkerSpeed,  Eigen::Matrix<double,2,2>& covariance){
        // Copmute the RPM of the wheels

        const double& left_speed  =   encoder_speed(0);
        const double& right_speed =  -encoder_speed(1);   // the - sign is because when the omega right is negative the walker is going forward

        //
        //const double r_c =  rwR_circ/60.0;  //2.0*(M_PI/60.0)*(0.305/2.0);  //rwR_circ/rad_s2rpm;  // coeff of right wheel
        //const double l_c =  rwL_circ/60.0;  //2.0*(M_PI/60.0)*(0.305/2.0);  //rwL_circ/rad_s2rpm;  // coeff of left wheel

        // TODO: added by me
        double wheelRadiusLeft = rwL_circ/(2*M_PI);
        double wheelRadiusRight = rwR_circ/(2*M_PI);

        walkerSpeed(0) =  (wheelRadiusRight*right_speed+wheelRadiusLeft*left_speed)/2.0;    // v
        walkerSpeed(1) =  (wheelRadiusRight*right_speed-wheelRadiusLeft*left_speed)/rearTrack;  // omega

        // Compute the covariance

        // Compute the static part of the encoder readings covariance static part (derivative of the equation respect to the tick_speed)
        Eigen::MatrixXd Q_encod_static(2,2);
        //Eigen::MatrixXd J_const(2,2);
        //J_const(0,0) = 0.5 * l_c;               // diff(walkerSpeed(0), left_speed)
        //J_const(0,1) = 0.5 * r_c;               // diff(walkerSpeed(0), right_speed)
        //J_const(1,0) = - l_c / rearTrack;       // diff(walkerSpeed(1), left_speed)
        //J_const(1,1) =   r_c / rearTrack;       // diff(walkerSpeed(1), right_speed)

        Eigen::MatrixXd J_var(2,3);
        J_var(0,0) = 0.5 * left_speed;               // diff(walkerSpeed(0), rwL_circ)
        J_var(0,1) = 0.5 * right_speed;              // diff(walkerSpeed(0), rwR_circ)
        J_var(0,2) = 0;                                            // diff(walkerSpeed(0), rearTrack)
        J_var(1,0) = - left_speed  / (rearTrack);     // diff(walkerSpeed(1), rwL_circ)
        J_var(1,1) =   right_speed / (rearTrack);    // diff(walkerSpeed(1), rwR_circ)
        J_var(1,2) = - walkerSpeed(1)/rearTrack;                   // diff(walkerSpeed(1), rearTrack)


        //const Eigen::MatrixXd J_const_t = J_const.transpose();
        const Eigen::MatrixXd J_var_t   = J_var.transpose();

        // Propagate the uncertanty of encoders and cov_parameters
        //covariance =    J_const * cov_encoders_speed * J_const_t + J_var * cov_param * J_var_t;
        covariance =    J_var * cov_param * J_var_t;

    }

    void Updatespeeds2DeltaTick() {
        Eigen::MatrixXd T(2,2);
        const double r_c =  encoder_ppr/rwR_circ;  // coeff of right wheel
        const double l_c =  encoder_ppr/rwL_circ;  // coeff of left wheel

        // delta tick left wheel from omega and v
        T(0,0) = l_c;
        T(0,1) = -l_c/2*rearTrack;


        // delta tick right wheel from omega and v
        T(1,0) = r_c;
        T(1,1) = r_c/2*rearTrack;


        H = Eigen::MatrixXd::Zero(2,5);
        H.bottomRightCorner(2,2)  =  T;
    }

};

#endif //CARRELLINO_WALKERKINEMATIC_H
