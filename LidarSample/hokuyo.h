////////////////////////////////////////////////////////////////////////////////
///
/// \file hokuyo.h
/// \brief Interface for connecting to Hokuyo sensors.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef HOKUYO_H
#define HOKUYO_H
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
    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Callback
    /// \brief Used to generate callbacks for subscribers to laser data. Overload
    ///        this callback and functions to recieve updated laser scans.
    ///
    ////////////////////////////////////////////////////////////////////////////////
    class Callback
    {
    public:
        Callback();
        virtual ~Callback() {}
        typedef std::set<Callback*> Set;
        /** Function called when new data becomes available from the laser,
            \param[in] Scan data in polar coordinates.
            \param[in] Time stamp in UTC since Jan 1, 1970. */
        virtual void ProcessLaserData(const std::vector<CvPoint3D32f> scan,
                                      const time_t timestamp) = 0;
    };
    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Hokuyo
    /// \brief Hokuyo is an interface class used to wrap the urg helper
    ///        library available on sourceforge. It can connect to
    ///        Hokuyo LIDARS and generate callbacks for each scan.
    ///
    ////////////////////////////////////////////////////////////////////////////////
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

    protected:
        /** This function should be called in a thread to continuously
            updated the scans
            \param[out] function writes to scan.
            \returns true on success, false on failure. */
        virtual bool GrabRangeData(std::vector<CvPoint3D32f>& scan);
        void CaptureThread();

        bool mConnectedFlag;
        bool mCaptureThreadFlag;
        int mBaudRate;
        void* mpDevice;
        long* mpHokuyoScan;
        int mHokuyoScanLength;
        int mHokuyoMinStep;
        int mHokuyoMaxStep;
        std::string mSerialPort;
        TiXmlDocument* mpDocument;
        boost::thread mCaptureThread;
        std::vector<CvPoint3D32f> mRangeScan;
        Callback::Set mCallbacks;
    };
}


#endif // HOKUYO_H
/* End of File */
