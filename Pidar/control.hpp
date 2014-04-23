////////////////////////////////////////////////////////////////////////////////
///
/// \file control.hpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef CONTROL_HPP
#define CONTROL_HPP
#include "commands.cpp"
#include "server.cpp"
#include "hokuyo.hpp"
#include "dynamixel.hpp"
#include "wiringPi.h"
#include <fstream>

#define HOKUYOSYNCPIN 17
#define LEDPIN 10
#define SHUTDOWNPIN 4


namespace Pidar
{
    class LaserCallback : public Laser::Callback
    {
    public:
        LaserCallback(){}
        ~LaserCallback(){}
        virtual void ProcessLaserData(const std::vector<Point3D>& scan,
                                      const time_t& timestamp)
        {
            mLaserScan = scan;
            mTimeStamp = timestamp;
            count++;
        }
        unsigned int count;
        std::vector<Point3D> mLaserScan;
        time_t mTimeStamp;
    };

    class DynamixelCallback : public Motor::Callback
    {
    public:
        DynamixelCallback(){}
        ~DynamixelCallback(){}
        virtual void ProcessServoData(const float& pos,
                                      const timespec& timestamp)
        {
            mMotorAngle = pos;
            mTimeStamp = timestamp;
        }
        float mMotorAngle;
        timespec mTimeStamp;
    };

    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Control
    /// \brief Used to handle control of all PIDAR components
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Control
    {
    public:
        ///
        ///  Controller Functions
        ///
        Control();
        ~Control();
        bool Initialize(); //laser, motor,

        ///
        ///  Motor Functions
        ///
        float GetMotorPositionDegrees();
        float GetMotorPreviousPositionDegrees();
        void SetMotorPreviousPositionDegrees(float val);
        bool isMotorConnected();
        void StartMotor(int rpm);
        void StopMotor();

        ///
        /// Laser Functions
        ///
        float GetLaserPositionPolar();
        std::vector<Point3D> GetLaserScan();
        void StopLaser();

        ///
        /// Scan Management Functions
        ///
        void AddToScanQueue(std::vector<Point3D> laserscan,
                                   float currentMotorPosition,
                                   float previousMotorPosition);
        bool StartISR();
        Laser::Hokuyo* GetLaserPtr()
        {
            return laser;
        }

    protected:
        Motor::Dynamixel* motor;
        Laser::Hokuyo *laser;
        std::vector<Point3D> mScan;
        time_t mLaserTimestamp;
        time_t mMotorTimestamp;
        float mStartScanAngle;
        float mStopScanAngle;
        DynamixelCallback mpMotorcallback;
        LaserCallback mpLasercallback;
        pcl_data mPointCloud;
    };

}

#endif
