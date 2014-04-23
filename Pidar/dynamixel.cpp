////////////////////////////////////////////////////////////////////////////////
///
/// \file dynamixel.cpp
/// \brief Interface for connecting to Dynamixel Motors. Wraps Dynamixel DXL SDK
///        available here:
///        http://support.robotis.com/en/software/dynamixelsdk.htm
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 1/28/13
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
/// Rx-24f api address reference:
/// http://support.robotis.com/en/product/dynamixel/rx_series/rx-24f.htm
////////////////////////////////////////////////////////////////////////////////
#include "dynamixel.hpp"

using namespace Motor;

Dynamixel::Dynamixel()
{
    mConnectedFlag = mProcessingThreadFlag = mCommandSpeedFlag =
    mFirstMotorReadFlag = false;
    mID = 1;
    mSerialPort = "/dev/ttyUSB0";
    mpDocument = new TiXmlDocument();
    mCommandSpeedRpm = mPresentPositionDegrees = mPreviousPositionDegrees = 0.0;
    mBaudRate = 0; // 2Mbps connection, fastest is 3Mbps.
}


Dynamixel::~Dynamixel()
{
    Shutdown();
}


/** Allows for loading of connection settings for a Dynamixel servo. */
bool Dynamixel::LoadSettings(const std::string& settings)
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
                if(elemName == "Motor")
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


/** Initializes a connection (serial) to a dynamixel servo. */
bool Dynamixel::Initialize()
{
    int deviceIndex;
    if(sscanf(mSerialPort.c_str(), "/dev/ttyUSB%d", &deviceIndex) <= 0)
    {
        std::cout << "Must use /dev/ttyUSB# for enumeration" << std::endl;
        return false;
    }

    if(dxl_initialize(deviceIndex, mBaudRate) != 0)
    {
        mConnectedFlag = true;
    }
    else
    {
        std::cout << "Failed to connect to Dynamixel" << std::endl;
        return false;
    }
    if(!StartCaptureThread())
    {
        std::cout << "Failed to start capture thread" << std::endl;
        return false;
    }
    std::cout << "Connected to Dynamixel Successfully" << std::endl;
    return true;
}


/** Shuts down connection to the servo (terminates thread). */
void Dynamixel::Shutdown()
{
    StopCaptureThread();
    if(IsConnected())
    {
        dxl_terminate();
        mConnectedFlag = false;
    }
}


/** Starts a thread for continuous capturing of sensor data. */
bool Dynamixel::StartCaptureThread()
{
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


/** Stops the thread for continuous capturing of sensor data. */
void Dynamixel::StopCaptureThread()
{
    mProcessingThreadFlag = false;
    mProcessingThread.join();
    mProcessingThread.detach();
}


/** Sets speed of motor in RPM given a direction
    \param[in] RPM to set (RX-24: 0~114 RPM, MX-28: 0~54 RPM)
    \param[in] true to move CW, false to move CCW*/
void Dynamixel::SetSpeedRpm(const float rpm, const bool clockwise)
{
    float val = rpm / MX28_RPM_PER_UNIT;
    if(val > 1023.0)
    {
        val = 1023.0;
    }
    if (clockwise)
    {
        val += 1024.0;
    }
    mMutex.lock();
    mCommandSpeedRpm = val;
    mCommandSpeedFlag = true;
    mMutex.unlock();
}


/** Get current position of servo
    \returns Position of motor in degrees */
float Dynamixel::GetPositionDegrees()
{
    boost::mutex::scoped_lock scopedLock(mMutex);
    return mPresentPositionDegrees;
}


/** Get previous position of servo
    \returns Previous position of motor in degrees */
float Dynamixel::GetPreviousPositionDegrees()
{
    boost::mutex::scoped_lock scopedLock(mMutex);
    return mPreviousPositionDegrees;
}


/** Sets previous position of servo at some earlier time.
    \param[in] Previous position of motor in degrees */
void Dynamixel::SetPreviousPositionDegrees(float val)
{
    boost::mutex::scoped_lock scopedLock(mMutex);
    mPreviousPositionDegrees = val;
}


/** Prints to screen the result of a wirte/read to the dynamixel */
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
    timespec t2;
    while(mProcessingThreadFlag)
    {
        if(IsConnected())
        {
            int CommStatus = 0;
            // Write target speed (if there is a new speed)
            {
                boost::mutex::scoped_lock lock (mMutex);
                if(mCommandSpeedFlag)
                {
                    dxl_write_word(mID, P_MOVING_SPEED_L, mCommandSpeedRpm);
                    CommStatus = dxl_get_result();
                    // Print reason of unsuccessful command.
                    if(CommStatus != COMM_RXSUCCESS)
                    {
                        PrintCommStatus(CommStatus);
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
            bool read = false;
            try
            {
                int recv = dxl_read_word(mID, P_PRESENT_POSITION_L );
                CommStatus = dxl_get_result();
                if(CommStatus == COMM_RXSUCCESS)
                {
                    boost::mutex::scoped_lock lock (mMutex);
                    mPresentPositionDegrees = recv * MX28_DEG_PER_UNIT;
                    read = true;
                }
                // Print reason of unsuccessful read command.
                else
                {
//                    PrintCommStatus(CommStatus);
                }
            }
            catch(std::exception e)
            {
                std::cout << e.what() << std::endl;
            }
            // Trigger callbacks (if there is a read)
            if(read)
            {
                boost::mutex::scoped_lock lock (mMutex);
                Callback::Set::iterator callback;
                for(callback = mCallbacks.begin();
                    callback != mCallbacks.end();
                    callback++)
                {
                    clock_gettime(CLOCK_REALTIME, &t2);
                    (*callback)->ProcessServoData(mPresentPositionDegrees,
                                                  t2);
                }
            }
            else
            {
                //std::cout << "No Read" << std::endl;
            }
        }
//        timespec sleep, remaining;
//        sleep.tv_sec = remaining.tv_sec = 0;
//        sleep.tv_nsec = 100000L; //100 microseconds
//        nanosleep(&sleep, &remaining);
        boost::this_thread::sleep(boost::posix_time::millisec(10));
    }
}

/* End of File */
