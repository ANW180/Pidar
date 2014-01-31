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
#ifndef DYNAMIXEL_HPP
#define DYNAMIXEL_HPP
#include <boost/thread.hpp>
#include <tinyxml.h>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <time.h>
#include <wiringPi.h>
#include <dxl_hal.h>
#include <dynamixel.h>

namespace Motor
{
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Callback
    /// \brief Used to generate callbacks for subscribers to servo data.
    ///        Overload this callback and functions to recieve updated laser
    ///        scans.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Callback
    {
    public:
        Callback() {}
        virtual ~Callback() {}
        typedef std::set<Callback*> Set;
        /** Function called when new data becomes available from the laser,
            \param[in] Position data. */
        virtual void ProcessServoData(const double pos) = 0;
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
        /** Allows for loading of connection settings for a Dynamixel servo. */
        virtual bool LoadSettings(const std::string& settings);
        /** Initializes a connection (serial) to a dynamixel servo. */
        virtual bool Initialize(const std::string& port);
        /** Shuts down connection to the servo (terminates thread). */
        virtual void Shutdown();
        /** Checks if connection to sensor is established
            \returns true if connected, false otherwise. */
        virtual bool IsConnected() const { return mConnectedFlag; }
        /** Starts a thread for continuous capturing of sensor data. */
        virtual bool StartCaptureThread();
        /** Stops the thread for continuous capturing of sensor data. */
        virtual void StopCaptureThread();
        /** Set goal position (0 to 4095 for 0.088 deg resolution)
            using RAM address 30 & 31 */
        virtual void SetPosition(const double val);
        /** Set speed of motor speed (RPM) using RAM address 32 & 33 */
        virtual void SetSpeed(const double rpm = 10);
        /** Set torque of motor (0 to 1023) using RAM address 34 & 35 */
        virtual void SetTorque(const int val = 1023);
        /** Get current position of servo (0 to 4095 for 0.088 deg resolution)
            using RAM address 36 & 37 */
        virtual double GetPresentPosition();
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
        TiXmlDocument* mpDocument;
        boost::thread mProcessingThread;
        Callback::Set mCallbacks;
    };
}

#endif // DYNAMIXEL_HPP
/* End of File */
