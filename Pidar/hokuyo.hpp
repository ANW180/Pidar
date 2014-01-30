////////////////////////////////////////////////////////////////////////////////
///
/// \file hokuyo.hpp
/// \brief Interface for connecting to Hokuyo sensors.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef HOKUYO_HPP
#define HOKUYO_HPP
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <tinyxml.h>
#include <urg_utils.h>
#include <urg_sensor.h>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <time.h>
#include <wiringPi.h>

namespace Laser
{
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Callback
    /// \brief Used to generate callbacks for subscribers to laser data.
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
            \param[in] Scan data in polar coordinates.
            \param[in] Time stamp in UTC */
        virtual void ProcessLaserData(const std::vector<CvPoint3D64f>& scan,
                                      const time_t& timestamp) = 0;
    };
    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Hokuyo
    /// \brief Hokuyo is an interface class used to wrap the urg helper
    ///        library available on sourceforge. It can connect to
    ///        Hokuyo LIDARS and generate callbacks for each scan.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Hokuyo
    {
    public:
        Hokuyo();
        ~Hokuyo();
        /** Allows for loading of connection settings for a Hokuyo lidar. */
        virtual bool LoadSettings(const std::string& settings);
        /** Initializes a connection (serial only) to a Hokuyo sensor. */
        virtual bool Initialize();
        /** Shuts down connection to the sensor (terminates thread). */
        virtual void Shutdown();
        /** Checks if connection to sensor is established
            \returns true if connected, false otherwise. */
        virtual bool IsConnected() const { return mConnectedFlag; }
        /** Starts a thread for continuous capturing of sensor data. */
        virtual bool StartCaptureThread();
        /** Stops the thread for continuous capturing of sensor data. */
        virtual void StopCaptureThread();
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
        virtual void CaptureThread();

        bool mConnectedFlag;
        bool mCaptureThreadFlag;
        int mBaudRate;
        void* mpDevice;
        long* mpHokuyoScan;
        int mHokuyoMinStep;
        int mHokuyoMaxStep;
        std::string mSerialPort;
        TiXmlDocument* mpDocument;
        boost::thread mCaptureThread;
        std::vector<CvPoint3D64f> mLaserScan;
        Callback::Set mCallbacks;
    };
}


#endif // HOKUYO_HPP
/* End of File */
