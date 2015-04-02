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
         * @brief The Callback class generates callbacks for subscribers to
         * motor data. Overload this class and corresponding functions to
         * recieve appropraite motor stats.
         */
        class Callback
        {
        public:
            typedef std::set<Callback*> Set;
            /**
             * @brief ProcessServoData Called when new servo position data is
             * available.
             * @param positionRadians
             * @param timestampUTC
             */
            virtual void ProcessServoData(const float& positionRadians,
                                          const timespec& timestampUTC) = 0;
        };
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
            virtual bool IsConnected() const { return mConnectedFlag; }
            /**
             * @brief StartCaptureThread
             * @returns Ture on start of servo IO thread, false otherwise.
             */
            virtual bool StartCaptureThread();
            /**
             * @brief StopCaptureThread Stops the serial IO thread.
             */
            virtual void StopCaptureThread();
            /**
             * @brief SetSpeedRpm Sets speed of Dynamixel in RPM.
             * @param desiredRPM desired speed of Dynamixel.
             * @param clockwise True if desired rotation is clockwise.
             */
            virtual void SetSpeedRpm(const float desiredRPM,
                                     const bool clockwise);
            /**
             * @brief PrintCommStatus Prints the result of a write/read command.
             * @param CommStatus
             */
            static void PrintCommStatus(int CommStatus);
            /**
             * @brief GetCurrentPositionDegrees
             * @return Current position value of Dynamixel in radians.
             */
            virtual float GetCurrentPositionRadians()
            {
                boost::mutex::scoped_lock lock(mMutex);
                return mCurrentPositionRad;
            }
            /**
             * @brief GetPreviousPositionDegrees
             * @return Previous set position of the motor in radians.
             */
            virtual float GetPreviousPositionRadians()
            {
                boost::mutex::scoped_lock lock(mMutex);
                return mPreviousPositionRad;
            }
            /**
             * @brief SetPreviousPositionDegrees
             * @param position The desired previous motor position to set (rad).
             */
            void SetPreviousPositionDegrees(float positionRad)
            {
                boost::mutex::scoped_lock lock(mMutex);
                mPreviousPositionRad = positionRad;
            }
            /**
             * @brief RegisterCallback
             * @param callback Callback to be registered.
             * @returns True on success, false otherwise.
             */
            virtual bool RegisterCallback(Callback* callback)
            {
                if(callback)
                {
                    mCallbacks.insert(callback);
                    return true;
                }
                return false;
            }
            /**
             * @brief RemoveCallback
             * @param callback Callback to be removed.
             * @returns True on success, false otherwise.
             */
            virtual bool RemoveCallback(Callback* callback)
            {
                if(callback)
                {
                    Callback::Set::iterator iter;
                    iter = mCallbacks.find(callback);
                    if(iter != mCallbacks.end())
                    {
                        mCallbacks.erase(callback);
                        return true;
                    }
                    return false;
                }
                return false;
            }
            /**
             * @brief ClearCallbacks
             */
            void ClearCallbacks()
            {
                mCallbacks.clear();
            }


        protected:
            /**
             * @brief ProcessingThread Serial IO thread function.
             */
            virtual void ProcessingThread();

            bool mConnectedFlag;
            bool mProcessingThreadFlag;
            int mBaudRate;
            int mID;
            std::string mSerialPort;
            boost::thread mProcessingThread;
            boost::mutex mMutex;
            Callback::Set mCallbacks;
            int mCommandSpeedRpm;
            float mCurrentPositionRad;
            float mPreviousPositionRad;
            bool mCommandSpeedFlag;
            bool mFirstMotorReadFlag;
        };
    }
}
/* End of File */
