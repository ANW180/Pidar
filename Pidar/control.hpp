////////////////////////////////////////////////////////////////////////////////
///
/// \file control.hpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
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
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Control
    /// \brief Used to handle control of all PIDAR components
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Control : public Laser::Callback,
                           Motor::Callback
    {
    public:
        virtual void ProcessLaserData(const Point3D::List& scan,
                                      const timespec& timestamp)
        {
            mLaserScan = scan;
            mLaserTimestamp = timestamp;
        }
        virtual void ProcessServoData(const float& pos,
                                      const timespec& timestamp)
        {
            mMotorAngle = pos;
            mMotorTimestamp = timestamp;
        }


        Control();
        ~Control();
        bool Initialize(); //laser, motor,
        void AddToScanQueue(Point3D::List laserscan,
                                   float currentMotorPosition,
                                   float previousMotorPosition);
        bool StartISR();

        Motor::Dynamixel* mMotor;
        Laser::Hokuyo* mLaser;
        Point3D::List mLaserScan;
        timespec mLaserTimestamp;
        timespec mMotorTimestamp;
        float mMotorAngle;
    };

}
/* End of File */
