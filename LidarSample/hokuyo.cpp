////////////////////////////////////////////////////////////////////////////////
///
/// \file hokuyo.cpp
/// \brief Interface for connecting to Hokuyo sensors.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include "hokuyo.h"

using namespace Laser;


Hokuyo::Hokuyo()
{
    mpDevice = new ::urg_t();
    mConnectedFlag = false;
    mCaptureThreadFlag = false;
    mpDevice = new ::urg_t();
    mpHokuyoScan = NULL;
    mHokuyoScanLength = mHokuyoMinStep = mHokuyoMaxStep = 0;
    mBaudRate = 115200;
    mSerialPort = "/dev/ttyACM0";
    mpDocument = new TiXmlDocument();
}


Hokuyo::~Hokuyo()
{
    Shutdown();
}


bool Hokuyo::LoadSettings(const std::string& settings)
{
    if(mpDocument->LoadFile(settings))
    {
        TiXmlElement* root = mpDocument->FirstChildElement();
        if(root)
        {
            for(TiXmlElement* elem = root->FirstChildElement();
                elem != NULL;
                elem = elem->NextSiblingElement())
            {
                std::string elemName = elem->Value();
                if(elemName == "Laser")
                {
                    mSerialPort = elem->Attribute("port");
                    mBaudRate = atoi(elem->Attribute("baud"));
                }
            }
        }
        return true;
    }

    return false;
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
    mpHokuyoScan = new long[mHokuyoScanLength];
    urg_step_min_max(urg, &mHokuyoMinStep, &mHokuyoMaxStep);
    urg_start_measurement(urg,
                          URG_DISTANCE,
                          URG_SCAN_INFINITY,
                          0);
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
    mCaptureThreadFlag = false;
    mCaptureThread.join();
    mCaptureThread.detach();

    if(IsConnected())
    {
        mCaptureThreadFlag = true;
        mCaptureThread = boost::thread(boost::bind(&Hokuyo::CaptureThread, this));
        return true;
    }

    return false;
}


void Hokuyo::StopCaptureThread()
{
    mCaptureThreadFlag = false;
    mCaptureThread.join();
    mCaptureThread.detach();
}


bool Hokuyo::GrabRangeData(std::vector<CvPoint3D32f>& scan)
{
    urg_t* urg = (urg_t*)mpDevice;
    if(IsConnected())
    {
        CvPoint3D32f point;
        long timestamp = 0;
        int index = 0;
        int scanCount = urg_get_distance(urg,
                                         mpHokuyoScan,
                                         &timestamp);
        if(scanCount <= 0)
        {
            std::cout << "Scanning Error: " << urg_error(urg) << std::endl;
        }

        for(int i = mHokuyoMinStep;
            i <= mHokuyoMaxStep;
            i++)
        {
            if(mpHokuyoScan[index] >= 20)
            {
                //Convert to meteres/radians.
                point.x = mpHokuyoScan[index]/1000.0;
                point.z = -1 * urg_step2rad(urg, i);
                //Save result
                scan.push_back(point);
                mRangeScan.push_back(point);
            }
            index++;
        }
        return true;
    }
    return false;
}


void Hokuyo::CaptureThread()
{
    while(mCaptureThreadFlag)
    {
        if(GrabRangeData(mRangeScan))
        {
            //Trigger Callback
        }
        boost::this_thread::sleep(boost::posix_time::millisec(1));
    }
}

/* End of File */
