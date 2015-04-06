/**
  \file Dynamixel.hpp
  \brief Interface for connecting to Dynamixel Motors. Wraps Dynamixel DXL SDK
        available here: http://support.robotis.com/en/software/dynamixelsdk.html
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "dxl/dxl_hal.h"
#include "dxl/dynamixel.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 8 Bit EEPROM Hex Addresses
#define CW_ANGLE_LIMIT_L      6
#define CW_ANGLE_LIMIT_H      7
#define CCW_ANGLE_LIMIT_L     8
#define CCW_ANGLE_LIMIT_H     9
// 8 Bit RAM Hex Addresses
#define GOAL_POSITION_L       30
#define GOAL_POSITION_H       31
#define PRESENT_POSITION_L    36
#define PRESENT_POSITION_H    37
#define MOVING                46
#define MOVING_SPEED_L        32
#define MOVING_SPEED_H        33
#define TORQUE_LIMIT_L        34
#define TORQUE_LIMIT_H        35
// Motor definitions
#define MX28_RPM_PER_UNIT       0.0572166665 // [0 - 1023] (0.114437928 rpm / 2)
#define MX28_RAD_PER_UNIT       0.00153398079
#define MAX_SPEED_RPM           25.0


namespace Pidar
{
    namespace Motor
    {
        /**
         * @brief The Dynamixel class is used to interface with Dynamixel servo
         * motors via a RS485 interface.
         */
        class Dynamixel
        {
        public:
            Dynamixel();
            ~Dynamixel();
            /**
             * @brief Initialize Starts a serial connection to a dynamixel.
             * @returns Ture on success, false otherwise.
             */
            virtual bool Initialize();
            /**
             * @brief Shutdown Releases serial connection with the servo.
             */
            virtual void Shutdown();
            /**
             * @brief IsConnected
             * @returns True if connected, false otherwise.
             */
            virtual bool IsConnected();
            /**
             * @brief SetSpeedRpm Sets speed of Dynamixel in RPM.
             * @param desiredRPM desired speed of Dynamixel.
             */
            virtual void SetSpeedRpm(const float desiredRPM);
            /**
             * @brief GetCurrentPositionDegrees
             * @return Current position value of Dynamixel in radians [0, 2Pi].
             */
            virtual float GetCurrentPositionRadians();
            /**
             * @brief PrintCommStatus Prints the result of a write/read command.
             * @param CommStatus
             */
            virtual void PrintCommStatus(int CommStatus);
        protected:
            int mID;
            int mBaudRate;
            int mCommandSpeedRpm;
            std::string mSerialPort;
            float mCurrentPositionRad;
            bool mConnectedFlag;
        };
    }
}
/* End of File */
