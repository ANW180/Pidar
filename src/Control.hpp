/**
  \file Control.hpp
  \brief Class for managing control of the Pidar components.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
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
#define SHUTDOWNBUTTONPIN 4


namespace Pidar
{
    /**
     * @brief InterruptService Global ISR function called on the rising edge
     * of the Hokuyo synchronous output pin.
     */
    void InterruptService(void);
    /**
     * @brief InterruptServiceStop Global ISR function called on the riising
     * edge of the shutdown button pin.
     */
    void InterruptServiceStop(void);


    /**
     * @brief The Control class manages acces to Pidar resources for generating
     * accurate 3D scans.
     */
    class Control : public Laser::Callback, Motor::Callback
    {
    public:
        /**
         * @brief Control
         */
        Control();
        /**
         * @brief Initialize
         * @returns True on success, false on failure.
         */
        bool Initialize();
        /**
         * @brief SetupWiringPi Calls wiring pi functions to enable correct
         * settings for all GPIO pins.
         * @returns True on success, false on failure.
         */
        bool SetupWiringPi();
        /**
         * @brief AddToScanQueue Handles 3D scan interpretation and buffering.
         * @param laserscan
         * @param currentMotorPosition
         * @param previousMotorPosition
         */
        void AddToScanQueue(Point3D::List laserscan,
                            float currentMotorPosition,
                            float previousMotorPosition);
        /**
         * @brief ProcessLaserData
         * @param polarScan
         * @param timestampUTC
         */
        virtual void ProcessLaserData(const Point3D::List& polarScan,
                                      const timespec& timestampUTC);
        /**
         * @brief ProcessServoData
         * @param positionRadians
         * @param timestampUTC
         */
        virtual void ProcessServoData(const float& positionRadians,
                                      const timespec& timestampUTC);


        Motor::Dynamixel* mpMotor;      // Pointer to the Dynamixel motor.
        Laser::Hokuyo* mpLaser;         // Pointer to the Hokuyo lidar.
        Point3D::List mLaserScan;       // A single 2D scan from the Hokuyo.
        timespec mLaserTimestamp;       // Current laser scan timestamp.
        timespec mMotorTimestamp;       // Current motor position timestamp.
        float mMotorAngle;              // Current motor angle.
    };

}
/* End of File */
