/**
  \file Dynamixel.cpp
  \brief Interface for connecting to Dynamixel Motors. Wraps Dynamixel DXL SDK
        available here: http://support.robotis.com/en/software/dynamixelsdk.html
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Dynamixel.hpp"

using namespace Pidar;
using namespace Motor;


Dynamixel::Dynamixel()
{
    mConnectedFlag = mProcessingThreadFlag = mCommandSpeedFlag =
            mFirstMotorReadFlag = false;
    mID = 1;
    mSerialPort = "/dev/ttyUSB0";
    mCommandSpeedRpm = mCurrentPositionRad = mPreviousPositionRad = 0.0;
    mBaudRate = 0; // 1Mbps connection
}


Dynamixel::~Dynamixel()
{
    Shutdown();
}


bool Dynamixel::Initialize()
{
    int deviceIndex;
    if(sscanf(mSerialPort.c_str(), "/dev/ttyUSB%d", &deviceIndex) <= 0)
    {
#ifdef DEBUG
        std::cout << "Must use /dev/ttyUSB# for enumeration" << std::endl;
#endif
        return false;
    }
    if(dxl_initialize(deviceIndex, mBaudRate) != 0)
    {
        mConnectedFlag = true;
    }
    else
    {
#ifdef DEBUG
        std::cout << "Failed to connect to Dynamixel" << std::endl;
#endif
        return false;
    }
    if(!StartCaptureThread())
    {
#ifdef DEBUG
        std::cout << "Failed to start capture thread" << std::endl;
#endif
        return false;
    }
#ifdef DEBUG
    std::cout << "Connected to Dynamixel Successfully" << std::endl;
#endif
    return true;
}


void Dynamixel::Shutdown()
{
    StopCaptureThread();
    if(IsConnected())
    {
        dxl_terminate();
        mConnectedFlag = false;
    }
}


bool Dynamixel::StartCaptureThread()
{
    // First detach the thread.
    mProcessingThreadFlag = false;
    mProcessingThread.join();
    mProcessingThread.detach();


    if(IsConnected())
    {
        mProcessingThreadFlag = true;
        mProcessingThread = boost::thread(
                    boost::bind(&Dynamixel::ProcessingThread, this));
        return true;
    }
    return false;
}


void Dynamixel::StopCaptureThread()
{
    mProcessingThreadFlag = false;
    mProcessingThread.join();
    mProcessingThread.detach();
}


void Dynamixel::SetSpeedRpm(const float rpm, const bool clockwise)
{
    // Threshold speed to within max allowed speed.
    float val = rpm;
    if(val > MAX_SPEED_RPM)
    {
        val = MAX_SPEED_RPM / MX28_RPM_PER_UNIT;
    }
    // Set to 0 if less than 0 (avoid divide by 0)
    else if(val <= 0.0)
    {
        val = 0.0;
    }
    else
    {
        val /= MX28_RPM_PER_UNIT;
    }
    // Ensure range is valid.
    if(val > 1023.0)
    {
        val = 1023.0;
    }
    // Add 1024 to offset for desired direction.
    if(clockwise)
    {
        val += 1024.0;
    }
    mMutex.lock();
    mCommandSpeedRpm = (int)val;
    mCommandSpeedFlag = true;
    mMutex.unlock();
}


void Dynamixel::PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        std::cout << "COMM_TXFAIL: TX packet failed" << std::endl;
        break;
    case COMM_TXERROR:
        std::cout << "COMM_TXERROR: Incorrect instruction packet" << std::endl;
        break;
    case COMM_RXFAIL:
        std::cout << "COMM_RXFAIL: RX packet failed" << std::endl;
        break;
    case COMM_RXWAITING:
        std::cout << "COMM_RXWAITING: Recieving status packet now" << std::endl;
        break;
    case COMM_RXTIMEOUT:
        std::cout << "COMM_RXTIMEOUT: No status packet" << std::endl;
        break;
    case COMM_RXCORRUPT:
        std::cout << "COMM_RXCORRUPT: Incorrect status packet" << std::endl;
        break;
    default:
        std::cout << "Unknown Error" << std::endl;
        break;
    }
}


void Dynamixel::ProcessingThread()
{
    timespec time;
    while(mProcessingThreadFlag)
    {
        if(IsConnected())
        {
            int CommStatus = 0;
            // Write target speed (if there is a new speed)
            {
                boost::mutex::scoped_lock lock(mMutex);
                if(mCommandSpeedFlag)
                {
                    dxl_write_word(mID, MOVING_SPEED_L, mCommandSpeedRpm);
                    CommStatus = dxl_get_result();
                    // Print reason of unsuccessful command.
                    if(CommStatus != COMM_RXSUCCESS)
                    {
#ifdef DEBUG
                        PrintCommStatus(CommStatus);
#endif
                    }
                    // Reset command flag to false if sending command
                    // succeeded so as to not flood serial commands.
                    else
                    {
                        mCommandSpeedFlag = false;
                    }
                }
            }
            // Read present position
            bool wasRead = false;
            try
            {
                int recv = dxl_read_word(mID, PRESENT_POSITION_L );
                CommStatus = dxl_get_result();
                if(CommStatus == COMM_RXSUCCESS)
                {
                    boost::mutex::scoped_lock lock(mMutex);
                    mCurrentPositionRad = recv * MX28_RAD_PER_UNIT;
                    wasRead = true;
                }
                // Print reason of unsuccessful read command.
                else
                {
#ifdef DEBUG
                    PrintCommStatus(CommStatus);
#endif
                }
            }
            catch(std::exception e)
            {
                std::cout << e.what() << std::endl;
            }
            // Trigger callbacks (if there is a read)
            if(wasRead)
            {
                boost::mutex::scoped_lock lock(mMutex);
                Callback::Set::iterator callback;
                for(callback = mCallbacks.begin();
                    callback != mCallbacks.end();
                    callback++)
                {
                    clock_gettime(CLOCK_REALTIME, &time);
                    (*callback)->ProcessServoData(mCurrentPositionRad, time);
                }
            }
            else
            {
#ifdef DEBUG
                std::cout << "No Read" << std::endl;
#endif
            }
        }
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }
}
/* End of File */
