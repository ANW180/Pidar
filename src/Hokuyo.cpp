/**
  \file Hokuyo.cpp
  \brief Interface for connecting to the Hokuyo UTM-30LX Lidar.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Hokuyo.hpp"
#include <time.h>

using namespace Pidar;
using namespace Laser;

long time_stamp_offset;


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
    mHokuyoMinStep = mErrorCount = 0;
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


static int pc_msec_time(void)
{
    static int is_initialized = 0;
    static struct timeval first_time;
    struct timeval current_time;

    long msec_time;
    if (!is_initialized)
    {
        gettimeofday(&first_time, NULL);
        is_initialized = 1;
    }
    gettimeofday(&current_time, NULL);
    msec_time =
            ((current_time.tv_sec - first_time.tv_sec) * 1000) +
            ((current_time.tv_usec - first_time.tv_usec) / 1000);

    return msec_time;
}


static long print_time_stamp(urg_t *urg, long time_stamp_offset)
{
    long sensor_time_stamp;
    long pc_time_stamp;
    long before_pc_time_stamp;
    long after_pc_time_stamp;
    long delay;
    urg_start_time_stamp_mode(urg);
    before_pc_time_stamp = pc_msec_time();
    //clock_gettime(CLOCK_MONOTONIC, &t1);
    sensor_time_stamp = urg_time_stamp(urg);    // delay of ~ 1ms.
    //computeAvg();
    after_pc_time_stamp = pc_msec_time();
    delay = (after_pc_time_stamp - before_pc_time_stamp) / 2;
    if (sensor_time_stamp < 0)
    {
        std::cout << "urg_time_stamp: " << urg_error(urg) << std::endl;
        return -1;
    }
    sensor_time_stamp -= time_stamp_offset;
    pc_time_stamp = pc_msec_time();
    urg_stop_time_stamp_mode(urg);
    std::cout << "PC: "
              << pc_time_stamp
              << " SENSOR: "
              << sensor_time_stamp
              << std::endl;
    return sensor_time_stamp - (pc_time_stamp - delay);
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
    time_stamp_offset = print_time_stamp(urg, 0);
//    for(int i = 0; i < 5; ++i)
//    {
//        print_time_stamp(urg, time_stamp_offset);
//    }
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
    //clock_gettime(CLOCK_MONOTONIC, &t3);
    while(mProcessingThreadFlag)
    {
        //clock_gettime(CLOCK_MONOTONIC, &t1);
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
            std::cout << timestamp - time_stamp_offset << std::endl;
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
                    point.SetX(mpHokuyoScan[index] / 1000.0); // Meters
                    point.SetIntensity(mpHokuyoScanIntensity[index]);
                }
                point.SetY(-1.0 * urg_step2rad(urg, i)); // Radians
                scan.push_back(point);
                index++;
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
                clock_gettime(CLOCK_REALTIME, &time);
                (*iter)->ProcessLaserData(mLaserScan, time);
            }
        }
    }
}
/* End of File */
