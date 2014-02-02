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
bool Dynamixel::Initialize(const std::string& port)
{
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

}


/** Set goal position (0 to 4095 for 0.088 deg resolution)
    using RAM address 30 & 31 */
void Dynamixel::SetPosition(const double val)
{
    //16 bit address (8 + 8)
    //30 (0X1E)Goal Position(L) Lowest byte of Goal Position RW
    //31 (0X1F)Goal Position(H) Highest byte of Goal Position RW

    //dxl_write_word((int id, int address, int value)
    int8_t firstbyte = (int)val & 0xFF;
    int8_t secondbyte = (int)val >> 8;

    dxl_write_word(mID, 0x1E, firstbyte ); //low
    dxl_write_word(mID, 0x1F, secondbyte ); //high
}


/** Set speed of motor speed (RPM) using RAM address 32 & 33 */
void Dynamixel::SetSpeed(const double rpm)
{
    int8_t firstbyte = (int)rpm & 0xFF;
    int8_t secondbyte = (int)rpm >> 8;
    dxl_write_word(mID, 0x20, firstbyte ); //low
    dxl_write_word(mID, 0x21, secondbyte ); //high
}


/** Set torque of motor (0 to 1023) using RAM address 34 & 35 */
void Dynamixel::SetTorque(const int val)
{
    int8_t firstbyte = (int)val & 0xFF;
    int8_t secondbyte = (int)val >> 8;
    dxl_write_word(mID, 0x22, firstbyte ); //low
    dxl_write_word(mID, 0x23, secondbyte ); //high

}


/** Get current position of servo (0 to 4095 for 0.088 deg resolution)
    using RAM address 36 & 37 */
double Dynamixel::GetPresentPosition()
{
    double val = 0.0;
    int8_t firstbyte = dxl_read_word( mID, 0x24 );
    int8_t secondbyte = dxl_read_word( mID, 0x25 );

    val = (secondbyte << 8)  | firstbyte;

    return val;
}


void Dynamixel::ProcessingThread()
{

}
/* End of File */
