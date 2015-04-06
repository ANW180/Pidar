/**
  \file Hokuyo.hpp
  \brief Interface for connecting to the Hokuyo UTM-30LX Lidar.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "Point3D.hpp"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <urg_utils.h>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <ctime>
#include <stdio.h>

namespace Pidar
{
    namespace Laser
    {
        /**
         * @brief The Callback class generates callbacks for subscribers of
         * laser data. Overload this class and corresponding functions to
         * receive appropriate laser data.
         */
        class Callback
        {
        public:
            typedef std::set<Callback*> Set;
            /**
             * @brief ProcessLaserData Invoked when new data becomes available
             * from the laser.
             * @param polarScan The captured scan in polar coordinates.
             * @param timestampUTC The UTC time the scan was captured.
             */
            virtual void ProcessLaserData(const Point3D::List& polarScan,
                                          const timespec& timestampUTC) = 0;
        };
        /**
          \class Hokuyo
          \brief Hokuyo is an interface class used to wrap the urg helper
                 library available on sourceforge. It can connect to
                 Hokuyo LIDARS and generate callbacks for each scan.
        */
        /**
         * @brief The Hokuyo class is used to interface with Hokuyo LIDARs via
         * a USB nterface.
         */
        class Hokuyo
        {
        public:
            Hokuyo();
            ~Hokuyo();
            /**
             * @brief Initialize Starts a serial connection to a Hokuyo.
             * @returns True on success, false otherwise.
             */
            virtual bool Initialize();
            /**
             * @brief Shutdown Releases serial connection with the Hokuyo.
             */
            virtual void Shutdown();
            /**
             * @brief IsConnected
             * @returns True if connected, false otherwise.
             */
            virtual bool IsConnected() const { return mConnectedFlag; }
            /**
             * @brief StartCaptureThread
             * @returns True on start of Hokuyo IO thread, false otherwise.
             */
            virtual bool StartCaptureThread();
            /**
             * @brief StopCaptureThread Stops the serial IO thread.
             */
            virtual void StopCaptureThread();
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
                }
                return false;
            }
            /**
             * @brief ClearCallbacks
             */
            void ClearCallbacks()
            {
                //TODO add mutex lock
                mCallbacks.clear();
            }
            /**
             * @brief GetErrorCount
             * @returns Number of read errors of serial data.
             */
            int GetErrorCount()
            {
                boost::mutex::scoped_lock lock(mMutex);
                return mErrorCount;
            }

        protected:
            /**
             * @brief ProcessingThread Serial IO tread function.
             */
            virtual void ProcessingThread();

            bool mConnectedFlag;
            bool mProcessingThreadFlag;
            int mBaudRate;
            void* mpDevice;
            long* mpHokuyoScan;
            unsigned short* mpHokuyoScanIntensity;
            int mHokuyoMinStep;
            int mHokuyoMaxStep;
            int mErrorCount;
            std::string mSerialPort;
            boost::mutex mMutex;
            boost::thread mProcessingThread;
            Point3D::List mLaserScan;
            Callback::Set mCallbacks;
        };
    }
}
/* End of File */
