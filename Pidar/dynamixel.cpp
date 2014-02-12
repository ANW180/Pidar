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
    mConnectedFlag = mProcessingThreadFlag = false;
    mID = 0;
    mSerialPort = "/dev/ttyUSB0";
    mpDocument = new TiXmlDocument();
    mServoCommand.clear();
    mServoFeedback.clear();
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
        std::cout << "Connected to Dynamixel Successfully" << std::endl;
        mConnectedFlag = true;
    }
    else
    {
        std::cout << "Failed to connect to Dynamixel" << std::endl;
        return false;
    }
    if(!StartCaptureThread())
    {
        return false;
    }
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


/** Set speed of motor speed (RPM) */
void Dynamixel::SetSpeedRpm(const double rpm)
{
    int word = (int)(rpm / 0.053);
    dxl_write_word(mID, P_MOVING_SPEED_L, word );
}


/** Set torque of motor (0 to 1023) */
void Dynamixel::SetTorqueLimit(const int val)
{
    dxl_write_word(mID, P_TORQUE_LIMIT_L, val );
}


/** Get current position of servo [-100,100]% */
double Dynamixel::GetPositionPercent()
{
    double val = 0.0;
    int pos = dxl_read_word(mID, P_PRESENT_POSITION_L );
    // 0 to 4095 for 0.088 deg resolution
    val = (pos - 2048) / 2048 * 100;
    if(val > 100.0)
    {
        val = 100.0;
    }
    if(val < -100.0)
    {
        val = -100.0;
    }
    return val;
}


void Dynamixel::ProcessingThread()
{

    while(mProcessingThreadFlag)
    {
        if(IsConnected())
        {

        }
        boost::this_thread::sleep(boost::posix_time::millisec(1));
    }
}


void Dynamixel::PrintErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    {
        std::cout << "Input voltage error" << std::endl;
    }
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

/* End of File */
