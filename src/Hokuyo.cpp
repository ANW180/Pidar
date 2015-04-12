/**
  \file Hokuyo.cpp
  \brief Interface for connecting to the Hokuyo UTM-30LX Lidar.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Hokuyo.hpp"

using namespace Pidar;
using namespace Laser;


//timespec t1, t2, t3;
//std::vector<double> tme;

//timespec diff(timespec start, timespec end)
//{
//    timespec temp;
//    if((end.tv_nsec - start.tv_nsec) < 0)
//    {
//        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
//        temp.tv_nsec = 1000000000L + end.tv_nsec - start.tv_nsec;
//    }
//    else
//    {
//        temp.tv_sec = end.tv_sec - start.tv_sec;
//        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
//    }
//    return temp;
//}


//void computeAvg()
//{
//    clock_gettime(CLOCK_MONOTONIC, &t2);
//    double t = diff(t1, t2).tv_sec + diff(t1, t2).tv_nsec * 1e-9;
//    tme.push_back(t);
//    double s = diff(t3, t2).tv_sec + diff(t3, t2).tv_nsec * 1e-9;
//    if(tme.size() > 0)
//    {
//        std::vector<double>::const_iterator it;
//        double total = 0.0;
//        for(it = tme.begin();
//            it != tme.end();
//            it++)
//        {
//            total += (*it);
//        }
//        total /= tme.size();
//        std::cout << "Avg Delay: "
//                  << total
//                  << " s"
//                  << std::endl;
//        clock_gettime(CLOCK_MONOTONIC, &t3);
//        tme.clear();
//    }
//}


Hokuyo::Hokuyo()
{
    mpDevice = new ::urg_t();
    mConnectedFlag = false;
    mProcessingThreadFlag = false;
    mpHokuyoScan = NULL;
    mpHokuyoScanIntensity = NULL;
    mHokuyoMinStep = mErrorCount = mTimeStamp = mTimeStampOffset = 0;
    mHokuyoMaxStep = 1080;
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


static long print_time_stamp(urg_t *urg, long timestampOffset)
{
    long hokuyoTimestamp;
    long pcTimestamp;
    long beforePCTimestamp;
    long afterPCTimestamp;
    long delay;

    urg_start_time_stamp_mode(urg);
    beforePCTimestamp = TimeMs();
    //clock_gettime(CLOCK_MONOTONIC, &t1);
    hokuyoTimestamp = urg_time_stamp(urg);    // delay of ~ 1ms.
    //computeAvg();
    afterPCTimestamp = TimeMs();
    delay = (afterPCTimestamp - beforePCTimestamp) / 2;
    if (hokuyoTimestamp < 0)
    {
        std::cout << "urg_time_stamp: " << urg_error(urg) << std::endl;
        return -1;
    }
    hokuyoTimestamp -= timestampOffset;
    pcTimestamp = TimeMs();
    urg_stop_time_stamp_mode(urg);
    std::cout << "PC: "
              << pcTimestamp
              << " SENSOR: "
              << hokuyoTimestamp
              << std::endl;
    return hokuyoTimestamp - (pcTimestamp - delay);
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
    mTimeStampOffset = print_time_stamp(urg, 0);
    //    for(int i = 0; i < 5; ++i)
    //    {
    //        print_time_stamp(urg, mTimeStampOffset);
    //    }
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
    urg_t* urg = (urg_t*)mpDevice;
    //clock_gettime(CLOCK_MONOTONIC, &t3);
    while(mProcessingThreadFlag)
    {
        //clock_gettime(CLOCK_MONOTONIC, &t1);
        if(IsConnected())
        {
            Point3D point;
            std::vector<Point3D> scan;
            int scanCount = urg_get_distance_intensity(urg,
                                                       mpHokuyoScan,
                                                       mpHokuyoScanIntensity,
                                                       &mTimeStamp);
            std::cout << mTimeStamp - mTimeStampOffset << std::endl;
            if(scanCount <= 0)
            {
                std::cout << "Scanning Error: " << urg_error(urg) << std::endl;
                mErrorCount++;
                continue;
            }
            // Iterate over all steps in the scan (1080).
            for(int i = mHokuyoMinStep;
                i <= mHokuyoMaxStep;
                i++)
            {
                // Convert to meteres/radians. Save into appropriate data
                // structure.
                {
                    boost::mutex::scoped_lock lock(mMutex);
                    point.SetX(mpHokuyoScan[i] / 1000.0); // Meters
                    point.SetIntensity(mpHokuyoScanIntensity[i]);
                }
                point.SetY(-1.0 * urg_step2rad(urg, i)); // Radians
                scan.push_back(point);
            }
            // Size of a full scan in 3D data structure is 553472 bytes.
            {
                boost::mutex::scoped_lock lock(mMutex);
                mLaserScan = scan;
            }
            mErrorCount = 0;
            // Trigger Callbacks
            // On average reads occur every 40 - 60 ms.
            //computeAvg();
            Callback::Set::iterator iter;
            for(iter = mCallbacks.begin();
                iter != mCallbacks.end();
                iter++)
            {
                boost::mutex::scoped_lock lock(mMutex);
                (*iter)->ProcessLaserData(mLaserScan, mTimeStamp);
            }
        }
    }
}
/* End of File */
