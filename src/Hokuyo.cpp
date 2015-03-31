/**
  \file Hokuyo.cpp
  \brief Interface for connecting to the Hokuyo UTM-30LX Lidar.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#include "Hokuyo.hpp"

using namespace Pidar;
using namespace Laser;


Hokuyo::Hokuyo()
{
    mpDevice = new ::urg_t();
    mConnectedFlag = false;
    mProcessingThreadFlag = false;
    mpHokuyoScan = NULL;
    mpHokuyoScanIntensity = NULL;
    mHokuyoMinStep = mHokuyoMaxStep = mErrorCount = 0;
    mBaudRate = 115200;
    mSerialPort = "/dev/ttyACM0";
}


Hokuyo::~Hokuyo()
{
    Shutdown();
    if(mpHokuyoScan)
    {
        delete[] mpHokuyoScan;
    }
    if(mpHokuyoScanIntensity)
    {
        delete[] mpHokuyoScanIntensity;
    }
}


bool Hokuyo::Initialize()
{
    urg_t* urg = (urg_t*)mpDevice;
    if(urg_open(urg,
                URG_SERIAL,
                mSerialPort.c_str(),
                mBaudRate) < 0 )
    {
        std::cout << "Connection Error: " << urg_error(urg) << std::endl;
        return false;
    }
    mConnectedFlag = true;
    if(mpHokuyoScan)
    {
        delete[] mpHokuyoScan;
        mpHokuyoScan = NULL;
    }
    mpHokuyoScan = new long[urg_max_data_size(urg)];
    if(mpHokuyoScanIntensity)
    {
        delete[] mpHokuyoScanIntensity;
        mpHokuyoScanIntensity = NULL;
    }
    mpHokuyoScanIntensity = new unsigned short[urg_max_data_size(urg)];
    urg_step_min_max(urg, &mHokuyoMinStep, &mHokuyoMaxStep);
    urg_start_measurement(urg,
                          URG_DISTANCE_INTENSITY,
                          URG_SCAN_INFINITY,
                          0);
    if(!StartCaptureThread())
    {
        std::cout << "Failed to start capture thread" << std::endl;
        return false;
    }
    std::cout << "Connected to Hokuyo Successfully" << std::endl;
    return true;
}


void Hokuyo::Shutdown()
{
    StopCaptureThread();
    urg_t* urg = (urg_t*)mpDevice;
    if(IsConnected())
    {
        urg_close(urg);
        mConnectedFlag = false;
    }
}


bool Hokuyo::StartCaptureThread()
{
    // First detach the thread.
    mProcessingThreadFlag = false;
    mProcessingThread.join();
    mProcessingThread.detach();

    if(IsConnected())
    {
        mProcessingThreadFlag = true;
        mProcessingThread = boost::thread(
                    boost::bind(&Hokuyo::ProcessingThread, this));
        return true;
    }
    return false;
}


void Hokuyo::StopCaptureThread()
{
    mProcessingThreadFlag = false;
    mProcessingThread.join();
    mProcessingThread.detach();
}


void Hokuyo::ProcessingThread()
{
    timespec time;
    urg_t* urg = (urg_t*)mpDevice;
    while(mProcessingThreadFlag)
    {
        if(IsConnected())
        {
            Point3D point;
            long timestamp = 0;
            int index = 0;
            std::vector<Point3D> scan;
            int scanCount = urg_get_distance_intensity(urg,
                                                       mpHokuyoScan,
                                                       mpHokuyoScanIntensity,
                                                       &timestamp);
            if(scanCount <= 0)
            {
                boost::mutex::scoped_lock lock(mMutex);
                std::cout << "Scanning Error: " << urg_error(urg) << std::endl;
                mErrorCount++;
                continue;
            }
            // Iterate over all steps in the scan (1080).
            for(int i = mHokuyoMinStep;
                i <= mHokuyoMaxStep;
                i++)
            {
                /** Add in all data, do filtering of bad points on client side
                // Verify points are within distance specifications of the laser
                if(mpHokuyoScan[index] >= urg->min_distance &&
                   mpHokuyoScan[index] <= urg->max_distance) */
                // Convert to meteres/radians. Save into appropriate data
                // structure.
                {
                    boost::mutex::scoped_lock lock(mMutex);
                    point.SetX(mpHokuyoScan[index]/1000.0);
                    point.SetIntensity(mpHokuyoScanIntensity[index]);
                }
                point.SetY(-1.0 * urg_step2rad(urg, i));
                scan.push_back(point);
                index++;
            }
            // Size of a full scan in 3D data structure is 553472 bytes.
            {
                boost::mutex::scoped_lock lock(mMutex);
                mLaserScan = scan;
                mErrorCount = 0;
            }
            //Trigger Callbacks
            Callback::Set::iterator iter;
            for(iter = mCallbacks.begin();
                iter != mCallbacks.end();
                iter++)
            {
                boost::mutex::scoped_lock lock(mMutex);
                clock_gettime(CLOCK_REALTIME, &time);
                (*iter)->ProcessLaserData(mLaserScan, time);
            }
        }
    }
}
/* End of File */
