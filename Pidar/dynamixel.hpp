////////////////////////////////////////////////////////////////////////////////
///
/// \file dynamixel.hpp
/// \brief Interface for connecting to Dynamixel Motors. Wraps Dynamixel DXL SDK
///        available here:
///        http://support.robotis.com/en/software/dynamixelsdk.htm
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 1/28/13
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
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
#include "dxl/dxl_hal.h"
#include "dxl/dynamixel.h"

// 8 Bit RAM Addresses
#define P_CW_ANGLE_LIMIT_L      6
#define P_CW_ANGLE_LIMIT_H      7
#define P_CCW_ANGLE_LIMIT_L     8
#define P_CCW_ANGLE_LIMIT_H     9
#define P_GOAL_POSITION_L       30
#define P_GOAL_POSITION_H       31
#define P_PRESENT_POSITION_L    36
#define P_PRESENT_POSITION_H    37
#define P_MOVING                46
#define P_MOVING_SPEED_L        32
#define P_MOVING_SPEED_H        33
#define P_TORQUE_LIMIT_L        34
#define P_TORQUE_LIMIT_H        35
#define MX28_RPM_PER_UNIT       0.053
#define MX28_DEG_PER_UNIT       0.088

namespace Motor
{
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Callback
    /// \brief Used to generate callbacks for subscribers to servo data.
    ///        Overload this callback and functions to recieve updated motor
    ///        position values.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Callback
    {
    public:
        Callback() {}
        virtual ~Callback() {}
        typedef std::set<Callback*> Set;
        /** Function called when new data becomes available from the laser,
            \param[in] Position data.
            \param{in] Time stamp in UTC */
        virtual void ProcessServoData(const float& pos,
                                      const timespec& timestamp) = 0;
    };
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Dynamixel
    /// \brief Dynamixel class is used to connect to the Dynamixel motors using
    ///        an FTDI serial interface.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Dynamixel
    {
    public:
        Dynamixel();
        ~Dynamixel();
        /** Initializes a connection (serial) to a dynamixel servo. */
        virtual bool Initialize();
        /** Shuts down connection to the servo (terminates thread). */
        virtual void Shutdown();
        /** Checks if connection to sensor is established
            \returns true if connected, false otherwise. */
        virtual bool IsConnected() const { return mConnectedFlag; }
        /** Starts a thread for continuous capturing of sensor data. */
        virtual bool StartCaptureThread();
        /** Stops the thread for continuous capturing of sensor data. */
        virtual void StopCaptureThread();
        /** Sets speed of motor in RPM given a direction
            \param[in] RPM to set (RX-24: 0~114 RPM, MX-28: 0~54 RPM)
            \param[in] true to move CW, false to move CCW*/
        virtual void SetSpeedRpm(const float rpm, const bool clockwise);
        /** Prints to screen the result of a wirte/read to the dynamixel */
        static void PrintCommStatus(int CommStatus);
        /** Get current position of servo
            \returns Position of motor in degrees*/
        virtual float GetCurrentPositionDegrees()
        {
            boost::mutex::scoped_lock lock(mMutex);
            return mCurrentPositionDeg;
        }
        /** Get previous position of servo
            \returns Previous position of motor in degrees */
        virtual float GetPreviousPositionDegrees()
        {
            boost::mutex::scoped_lock lock(mMutex);
            return mPreviousPositionDeg;
        }
        /** Sets previous position of servo at some earlier time.
            \param[in] Previous position of motor in degrees */
        void SetPreviousPositionDegrees(float val)
        {
            boost::mutex::scoped_lock lock(mMutex);
            mPreviousPositionDeg = val;
        }
        /** Method to register callbacks */
        virtual bool RegisterCallback(Callback* cb)
        {
            //TODO add mutex lock
            mCallbacks.insert(cb);
            return true;
        }
        /** Method to remove callbacks */
        virtual bool RemoveCallback(Callback* cb)
        {
            //TODO add mutex lock
            Callback::Set::iterator iter;
            iter = mCallbacks.find(cb);
            if(iter != mCallbacks.end())
            {
                mCallbacks.erase(cb);
                return true;
            }
            return false;
        }
        /** Method to clear callbacks */
        void ClearCallbacks()
        {
            //TODO add mutex lock
            mCallbacks.clear();
        }


    protected:
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
        float mCurrentPositionDeg;
        float mPreviousPositionDeg;
        bool mCommandSpeedFlag;
        bool mFirstMotorReadFlag;
    };
}
/* End of File */
