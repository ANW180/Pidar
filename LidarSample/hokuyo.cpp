#include "hokuyo.h"

using namespace Sensor;

Hokuyo::Hokuyo()
{
    mpDevice = new ::urg_t();
    mConnectedFlag = false;
    mCaptureThreadFlag = false;
    mpDocument = NULL;
    mpDevice = NULL;
    mpHokuyoScan = NULL;
    mHokuyoScanLength = mHokuyoMinStep = mHokuyoMaxStep = 0;
    mBaudRate = 115200;
    mSerialPort = "/dev/ttyACM0";
}

Hokuyo::~Hokuyo()
{
    StopCaptureThread();
}

bool Hokuyo::LoadSettings(const std::string &settings)
{
    if(mpDocument->LoadFile(settings))
    {

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

bool Hokuyo::GrabRangeData()
{
    urg_t* urg = (urg_t*)mpDevice;
    if(IsConnected())
    {
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
                //Convert to cartesian/meters and store in data structure.
            }
            index++;
        }
        return true;
    }
    return false;
}

void Hokuyo::CaptureThread()
{

}

/* End of File */
