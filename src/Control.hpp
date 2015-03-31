/**
  \file Control.hpp
  \brief Class for managing control of the Pidar components.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#pragma once
#include "CommandReceiver.hpp"
#include "Server.hpp"
#include "Hokuyo.hpp"
#include "Dynamixel.hpp"
#include "wiringPi.h"
#include <fstream>

#define HOKUYOSYNCPIN 17
#define LEDPIN 10
#define SHUTDOWNPIN 4


namespace Pidar
{
    /**
      \class Control
      \brief Handles control of all Pidar components.
    **/
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
