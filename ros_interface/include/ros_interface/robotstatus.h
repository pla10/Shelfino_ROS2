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
