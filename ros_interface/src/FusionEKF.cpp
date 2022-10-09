#include <FusionEKF.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

    /*
     * Initialize state and state covariance
     * 1) x coordinate [m]
     * 2) y coordinate [m]
     * 3) head direction [rad]
     * 4) speed [m/s]
     * 5) head rate [rad/sec]
     */

    vehicle_s = MatrixXd::Zero(localizationMatrix::STATE_SIZE,1);  // Initial state is in the origin [0,0...]

    // Set the initial uncertanty at some high value i.e. 100m of uncertanty x,y, 100 rad, 100 m/s of velocity, 100 rad/s of omega
    vehicle_P = MatrixXd::Identity(localizationMatrix::STATE_SIZE, localizationMatrix::STATE_SIZE)*(100*100);



    Q_pred = MatrixXd::Identity(2,2);         // Covariance of the unmodeled quantities i.e.  acceleration and angular acceleration
    Q_pred(0,0) = 0.5;   // [(m/s^2)^2] linear accleratino convariance
    Q_pred(1,1) = 0.1;   // [(rad/s^2)^2] angolar accleratino convariance

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() { }

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    //SDEBUG("FusionEKF::ProcessMeasurement");
    std::unique_lock<std::mutex> lock(process_measure_mtx);  // it's not possible to serve 2 measure at the same time

    /* Mechanism to prevent stucks if walker hardware restarts _____ THIS IS TEMPORARY! */
    static int counter_old = 0;
    static const int iteration_before_reset = 100;
    /* --- */


	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
        //SDEBUG("FusionEKF::ProcessMeasurement KF initialization");
        // First time we have no clue on the time, get it.
        int actual_timestamp = measurement_pack.timestamp_;
        previous_timestamp_ = actual_timestamp;
        is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
	int actual_timestamp = measurement_pack.timestamp_;

	//compute the time elapsed between the current and previous measurements
    if(actual_timestamp>previous_timestamp_){
        //SDEBUG("FusionEKF::ProcessMeasurement Prediction");
        const double dt = ((double)actual_timestamp - (double)previous_timestamp_) / 1000.0;    //dt - expressed in seconds

        // Predict the state at time of the measure using the nonlinear prediction
        this->nonlinearStatePrediction(vehicle_s,vehicle_P,dt);
        /* Mechanism to prevent stucks if walker hardware restarts _____ THIS IS TEMPORARY! */
        counter_old = 0;
        /* --- */
        // Update time
        previous_timestamp_ = actual_timestamp;
    }else{
        //const double dt = - ((double)(previous_timestamp_ - (double)actual_timestamp) / 1000.0);    //dt - expressed in seconds
        //SDEBUG("FusionEKF::ProcessMeasurement propagate measurement taken"<<dt<<"[s] ago")
        // "Propagate measurement ahead"
        if(measurement_pack.sensor_type_ == MeasurementPackage::QR){
            //std::cout<<" ----> dt: "<<dt*1000<<" [ms]"<<std::endl;
            //SDEBUG("FusionEKF::ProcessMeasurement is a QR measure and was taken "<<uint64_t(dt*1000)<<"[s] ago");
            // TODO: propagate measurement ahead
        }else{
            /* Mechanism to prevent stucks if walker hardware restarts _____ THIS IS TEMPORARY! */
            if(++counter_old >= iteration_before_reset) previous_timestamp_ = actual_timestamp;
            /* --- */
            //SDEBUG("FusionEKF::ProcessMeasurement was taken "<<dt<<"[s] ago and discarded");
            //previous_timestamp_ = actual_timestamp;
            return; // Discard old measure,  who cares of the past encoders readings
        }
    }



	/*****************************************************************************
	 *  Update
	 ****************************************************************************/
    // Switch on the type of the measure
    switch( measurement_pack.sensor_type_ ) {

        /*************** ENCODER *******************/
        case MeasurementPackage::ENCODER:
        {
            //SDEBUG("FusionEKF::ProcessMeasurement Update for Encoder Measure");
            // Convert the wheels rotational speed in v and head rate of the walker
            Eigen::Matrix<double,2,1> walkerSpeed;
            Eigen::Matrix<double,2,2> walkerSpeed_cov;
            //std::cout<<"Right speed [RPM]: "<<measurement_pack.raw_measurements_(0)<<std::endl;
            //std::cout<<"left speed [RPM]:  "<<measurement_pack.raw_measurements_(1)<<std::endl;

            kinematic.motorSpeed2walkerSpeed(measurement_pack.raw_measurements_, walkerSpeed, walkerSpeed_cov);
            //std::cout<<"Right speed [rad/s]: "<<walkerSpeed[0]<<std::endl;
            //std::cout<<"left speed [rad/s]:  "<<walkerSpeed[1]<<std::endl;

            //kinematic.tickspeed2walkerSpeed(measurement_pack.raw_measurements_, walkerSpeed, walkerSpeed_cov);

            // We measure the last 2 element of the state
            Eigen::Matrix<double,2,localizationMatrix::STATE_SIZE> H = MatrixXd::Zero(2, localizationMatrix::STATE_SIZE);
            H.bottomRightCorner(2, 2) = MatrixXd::Identity(2, 2);

            // 0,0,0,1,0
            // 0,0,0,0,1

            // Update the kalman filter
            //lin_kf.UpdateSpeed(vehicle_s,vehicle_P,walkerSpeed,walkerSpeed_cov);
            lin_kf.Update(vehicle_s,vehicle_P,walkerSpeed, H, walkerSpeed_cov);
                     //5x1,5x5,2x1,2x5,2x2
        }break;

        /*************** QR *******************/
        case MeasurementPackage::QR:
        {
            //SDEBUG("FusionEKF::ProcessMeasurement Update for QR Measure");
            // We measure the first 3 element of the state
            Eigen::Matrix<double,3,1> walkerPose = measurement_pack.raw_measurements_;
            walkerPose(2) = normalizeAngle(walkerPose(2));
            Eigen::Matrix<double,3,localizationMatrix::STATE_SIZE> H = MatrixXd::Identity(3, localizationMatrix::STATE_SIZE);

            //1 0 0 0 0
            //0 1 0 0 0
            //0 0 1 0 0

            //x         0 0 0 0 0
            //y         0 0 0 0 0
            //theta     0 0 1 0 0
            //v         0 0 0 0 0
            //omega     0 0 0 0 0
            
            //x         1 0 0 0 0
            //y         0 0 0 0 0
            //theta     0 0 0 0 0
            //v         0 0 0 0 0
            //omega     0 0 0 0 0

            const double sigmaPos = 0.3;   // 3 cm of sigma in placing qr both in x and y
            const double sigmaAng = 15*M_PI/180; // 15 degree of sigma in placing qr

            Eigen::Matrix<double,3,3> qr_readings_cov;

            qr_readings_cov << sigmaPos*sigmaPos,                0  , 0,
                                               0, sigmaPos*sigmaPos , 0,
                                               0,                 0 , sigmaAng*sigmaAng;


            // Update the kalman filter
            lin_kf.Update(vehicle_s,vehicle_P,walkerPose, H, qr_readings_cov);

        }break;
        /******/
        default:
        {
            //SDEBUG("FusionEKF::ProcessMeasurement measure is discarded");
        }break;
    }
}



void FusionEKF::nonlinearStatePrediction(localizationMatrix::StateVec_t& s, localizationMatrix::StateCovMatrix_t& P, const double dt){
    //SDEBUG("FusionEKF::nonlinearStatePrediction");
	const double x0     = s(0);
	const double y0     = s(1);
	const double theta0 = s(2);
    const double v0     = s(3);
    const double omega0 = s(4);

	// Init the Jacobian matrix
    localizationMatrix::StateCovMatrix_t J = MatrixXd::Identity(5,5);


    Eigen::Matrix<double,localizationMatrix::STATE_SIZE,2> J_unmodel = MatrixXd::Identity(5,2); // Error due to the unmodeled contribute of the acceleration, and angular acceleration

    // Compute some common variable that can be computed once (due to computational cost)
    const long double cos0 = cos(theta0);    
    const long double sin0 = sin(theta0);

    const long double cosDt0 = cos0 * dt;
    const long double sinDt0 = sin0 * dt;

    const long double dx0 = v0 * cosDt0;
    const long double dy0 = v0 * sinDt0;

    const long double dtheta0 = omega0 * dt;
    
    const double dt2 = dt*dt;
    //const double dt3 = dt2*dt;
    
    //const double omega20 = omega0*omega0;

    const double d2theta_6 = dtheta0 * dtheta0 / 6;
    const double dtheta_2  = dtheta0 / 2;


    /// X1 (for understand the following formulas look the PDF in the localization folder)
    //std::cout<<"X1"<<std::endl;
    
    const double x1 = x0 + dx0 - dy0 * dtheta_2 - dx0 * d2theta_6;

    //Sulla modellazione dell'errore nel caso di 7 stati, diventa più pratico tarare manualmente il filtro.
    //Al posto delle formule metti dei valori e lo tari.
    //Lo svantaggio è che devi mettere 7 stati e rifarmi tutta la dinamica.
    //Questa è l'ultima spiaggia
    //V e Omega li stimo bene con gli encoder, e Theta con la IMU.

    // x covariance propagation
    J(0,0) = 1;  //diff(x1,x0)
    J(0,1) = 0;  //diff(x1,y0)
    J(0,2) = - dy0 - dx0  * dtheta_2 + dy0 * d2theta_6;  //diff(x1,theta0)
    J(0,3) =   cosDt0 - sinDt0 * dtheta_2 + cosDt0 * d2theta_6;  // diff(x1,v0)
    J(0,4) = - dy0 * dt / 2 - dx0 * dtheta0 * dt;  // diff(x1,omega0)

    // Model error
    J_unmodel(0,0) =  dt * (3 * cosDt0 - 2 * dtheta0 * sinDt0) / 6;      // diff(x1,v_dot)
    J_unmodel(0,1) = -dt2 * dy0/6;                             // diff(x1,omega_dot)


    /// Y1
    //std::cout<<"Y1"<<std::endl;
    const double y1 = y0 + dy0 + dx0 * dtheta_2 - dy0 * d2theta_6;

    // y covariance propagation
    J(1,0) = 0;  //diff(y1,x0)
    J(1,1) = 1;  //diff(y1,y0)
    J(1,2) = dx0 - dy0 * dtheta_2 - dx0 * d2theta_6;   //diff(y1,theta0)
    J(1,3) = sinDt0 + cosDt0 * dtheta_2 - sinDt0 * d2theta_6;  // diff(y1,v0)
    J(1,4) = dx0 * dt / 2 - dy0 * dtheta0 * dt;                            // diff(y1,omega0)

    // Model error
    J_unmodel(1,0) = dt * (3*sinDt0 - 2 * dtheta0 * cosDt0) / 6;   // diff(y1,v_dot)
    J_unmodel(1,1) = dt2 * dx0 / 6;                           // diff(y1,omega_dot)


    /// THETA1
    //std::cout<<"Theta1"<<std::endl;
    const double theta1 = theta0 + dtheta0; //theta1 = theta0 + omega0*dt + 0.5*accAng*dt^2

    // theta covariance propagation
    J(2,0) = 0;   // diff(theta1,x0)
    J(2,1) = 0;   // diff(theta1,y0)
    J(2,2) = 1;   // diff(theta1,theta0)
    J(2,3) = 0;   // diff(theta1,v0)
    J(2,4) = dt;  // diff(theta1,omega0)

    // Model error
    J_unmodel(2,0) = 0;           // diff(theta1,v_dot)
    J_unmodel(2,1) = 0.5*dt2;     // diff(theta1,omega_dot)


    /// V1
    //std::cout<<"V1"<<std::endl;
    const double v1 = v0; //v(t) = cost v1 = v0 + a*dt -> se acc = cost

    // theta covariance propagation
    J(3,0) = 0;   // diff(v1,x0)
    J(3,1) = 0;   // diff(v1,y0)
    J(3,2) = 0;   // diff(v1,theta0)
    J(3,3) = 1;   // diff(v1,v0)
    J(3,4) = 0;   // diff(v1,omega0)

    // Model error
    J_unmodel(3,0) = dt;  // diff(v1,v_dot)
    J_unmodel(3,1) = 0;   // diff(v1,omega_dot)


    /// OMEGA1
    //std::cout<<"Omega1"<<std::endl;
    const double omega1 = omega0; //come la velocità

    // theta covariance propagation
    J(4,0) = 0;   // diff(omega1,x0)
    J(4,1) = 0;   // diff(omega1,y0)
    J(4,2) = 0;   // diff(omega1,theta0)
    J(4,3) = 0;   // diff(omega1,v0)
    J(4,4) = 1;   // diff(omega1,omega0)

    // Model error
    J_unmodel(4,0) = 0;  // diff(omega1,v_dot)
    J_unmodel(4,1) = dt;   // diff(omega1,omega_dot)

    ///ACC LIN (stato 6)
    /// acc1 = acc0
    /// ACC ANG (stato 7)
    /// accAng1 = accAng0

    /// Update state
	s(0) = x1;
	s(1) = y1;
	s(2) = theta1;
	s(3) = v1;
	s(4) = omega1;


	/// Update covariance

    const Eigen::Matrix<double,localizationMatrix::STATE_SIZE,localizationMatrix::STATE_SIZE> Jt = J.transpose();
    const Eigen::Matrix<double,2,localizationMatrix::STATE_SIZE> J_unmodelt = J_unmodel.transpose();

    Eigen::Matrix<double,localizationMatrix::STATE_SIZE,localizationMatrix::STATE_SIZE> R = Eigen::MatrixXd::Identity(5,5);
    R(3,3) = 0.1;
    R(4,4) = 0.1;
    R = 1e-6*R;
	P = J*P*Jt + R + J_unmodel*Q_pred*J_unmodelt;

    //P = J*P*Jt + J_unmodel*Q_pred*J_unmodelt;
	return;

}

void FusionEKF::InitializeFilter(const localizationMatrix::StateVec_t &x_, const localizationMatrix::StateCovMatrix_t &P_) {
    //SDEBUG("FusionEKF::InitializeFilter ");
    std::unique_lock<std::mutex> lk_mtx(process_measure_mtx);  // stop ProcessMeasurement and set the new state
    vehicle_s = x_;
    vehicle_P = P_;
}

bool FusionEKF::InitializeFilterJSON(const nlohmann::json &j){

    try{
        const double x               = j.at("x").get<double>();      // x state [m]
        const double y               = j.at("y").get<double>();      // y state [m]
        const double theta           = j.at("theta").get<double>();  // heading state [m]
        
        const double sigmaV = 2;  // cov in [m/s], at 68% the walker is going at a speed between -2 and +2 m/s
        const double sigmaV2 = sigmaV*sigmaV;
        const double sigmaOmega = M_PI;  // cov in [rad/s], at 68% the walker is going at a speed between -3 and +3 rad/s
        const double sigmaOmega2 = sigmaOmega*sigmaOmega;

        Eigen::Matrix<double, 5, 1> s0 = Eigen::VectorXd::Zero(5);
        Eigen::Matrix<double, 5, 5> P0 = Eigen::MatrixXd::Identity(5,5);


        s0 << x,y,theta,0,0;   // estimate value of v and omega are zeros

        //sono settati valori molto alti nella matrice apposta perche se si incontra un qr gli si dara parecchia fiducia xD
        P0 << 100.0,      0,          0,      0,          0,
                   0,100.0,          0,      0,          0,
                   0,      0,10.0,      0,          0,
                   0,      0,          0, sigmaV2,          0,
                   0,      0,          0,      0,sigmaOmega2;

        std::cout <<"Initial state:       "<< s0.transpose() << std::endl;
        std::cout <<"Initial covariance:  "<< P0 << std::endl;
        InitializeFilter(s0,P0);
    } catch(std::exception & e){
        return false;
    }
    return true;
}

void FusionEKF::getLastFilterInformation(int &time, localizationMatrix::StateVec_t &state, localizationMatrix::StateCovMatrix_t &covariance) {
    //SDEBUG("FusionEKF::getLastFilterInformation ");
    std::unique_lock<std::mutex> lk_mtx(process_measure_mtx);
    time = previous_timestamp_;
    state = vehicle_s;
    covariance = vehicle_P;
}

double FusionEKF::normalizeAngle(double theta) {
    //std::cout<<"theta IN: "<<theta<<std::endl;
    while((theta-vehicle_s(2)) > M_PI){
        theta = theta - 2*M_PI;
    }
    while((theta-vehicle_s(2)) < (-M_PI) ){
        theta = theta + 2*M_PI;
    }

    //std::cout<<"theta OUT: "<<theta<<std::endl;
    return theta;
}
