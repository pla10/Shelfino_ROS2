#ifndef HARDWARESTATUSTYPE_H
#define HARDWARESTATUSTYPE_H

#include <utility>
#include <vector>
#include <mutex>

namespace RobotStatus {

    struct LocalizationData{
        double x=0;
        double y=0;
        double theta=0;
        double locTimer=0;

        LocalizationData(){}

        LocalizationData(const LocalizationData & a){
            x = a.x;
            y = a.y;
            theta = a.theta;
            locTimer = a.locTimer;
        }
    };

    struct WheelSpeed {
        double dTick = 0.0;
        //double lastTick;
        double lastTime = 0.0;
        bool init = false;
    };

    struct OdometryData{
        double last_odom_update = 0;

        double theta = 0;

        // Pose
        double pos_x=0;
        double pos_y=0;
        double pos_z=0;
        double orient_x=0;
        double orient_y=0;
        double orient_z=0;
        double orient_w=0;
        std::vector<double> pose_cov;

        // Twist
        double twist_lin_x=0;
        double twist_lin_y=0;
        double twist_lin_z=0;
        double twist_ang_x=0;
        double twist_ang_y=0;
        double twist_ang_z=0;
        std::vector<double> twist_cov;

        OdometryData(){}

        OdometryData(const OdometryData & a){
            pos_x=a.pos_x;
            pos_y=a.pos_y;
            pos_z=a.pos_z;
            orient_x=a.orient_x;
            orient_y=a.orient_y;
            orient_z=a.orient_z;
            orient_w=a.orient_w;
            pose_cov=a.pose_cov;
            twist_lin_x=a.twist_lin_x;
            twist_lin_y=a.twist_lin_y;
            twist_lin_z=a.twist_lin_z;
            twist_ang_x=a.twist_ang_x;
            twist_ang_y=a.twist_ang_y;
            twist_ang_z=a.twist_ang_z;
            twist_cov=a.twist_cov;
            last_odom_update=a.last_odom_update;
            theta=a.theta;
        }
    };

    struct RearWheelData{
        double omega = 0;
        double current = 0;
        double ticks = 0;

        RearWheelData(){}
        RearWheelData(const RearWheelData &a){
            omega = a.omega;
            current = a.current;
            ticks = a.ticks;
        }
    };

    struct TrolleyData{
        double angle = 0;
        double force = 0;
        double trolleyTimer=0;
        int powerEnable;

        TrolleyData(){}
        TrolleyData(const TrolleyData &a){
            angle = a.angle;
            force = a.force;
            trolleyTimer = a.trolleyTimer;
            powerEnable = a.powerEnable;
        }
    };

    struct TrackerData{

    };

    struct HardwareData{
        RearWheelData rightWheel;
        RearWheelData leftWheel;
        double speed = 0;
        double omega = 0;
        double hardwareTimer=0;

        HardwareData(){}
        HardwareData(const HardwareData &a):
        rightWheel(a.rightWheel),
        leftWheel(a.leftWheel){
            speed = a.speed;
            omega = a.omega;
            hardwareTimer = a.hardwareTimer;
        }
    };

    struct LidarDatum{
        double angle;
        double distance;
        double x;
        double y;
        bool isSafe;

        LidarDatum(){}
        LidarDatum(const LidarDatum &a){
            angle = a.angle;
            distance = a.distance;
            x = a.x;
            y = a.y;
            isSafe = a.isSafe;
        }
    };

    struct LidarData{
        //std::vector<std::pair<double,double>> data;
        //std::vector<std::pair<double,double>> cartesianData; // (x-y) data w.r.t the vehicle reference frame
        std::vector<LidarDatum> datum;
        double lidarTimer = 0;
        double x_offset = 0; //x mounting offset w.r.t the vehicle reference frame
        double y_offset = 0; //y mounting offset w.r.t the vehicle reference frame

        LidarData(){}
        LidarData(const LidarData &a){
            datum = a.datum;
            lidarTimer = a.lidarTimer;
            x_offset = a.x_offset;
            y_offset = a.y_offset;
        }

        void setMountingPosition(double x, double y){
            x_offset = x;
            y_offset = y;
        }
    };

}

#endif // HARDWARESTATUSTYPE_H
