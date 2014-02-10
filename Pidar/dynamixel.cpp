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

    int word = (int)val;//translation?
    int lowbyte = dxl_get_lowbyte(word);
    int highbyte = dxl_get_highbyte(word);
    dxl_write_word(mID, 30, lowbyte ); //low
    dxl_write_word(mID, 31, highbyte ); //high
}


/** Set speed of motor speed (RPM) using RAM address 32 & 33 */
void Dynamixel::SetSpeed(const double rpm)
{
    int word = (int)rpm;//translation?
    int lowbyte = dxl_get_lowbyte(word);
    int highbyte = dxl_get_highbyte(word);


    dxl_write_word(mID, 32, lowbyte ); //low
    dxl_write_word(mID, 33, highbyte ); //high
}


/** Set torque of motor (0 to 1023) using RAM address 34 & 35 */
void Dynamixel::SetTorque(const int val)
{
    int word = (int)val;//translation?
    int lowbyte = dxl_get_lowbyte(word);
    int highbyte = dxl_get_highbyte(word);

    dxl_write_word(mID, 34, lowbyte ); //low
    dxl_write_word(mID, 35, highbyte ); //high

}


/** Get current position of servo (0 to 4095 for 0.088 deg resolution)
    using RAM address 36 & 37 */
double Dynamixel::GetPresentPosition()
{
    double val = 0.0;
    int lowbyte = dxl_read_word( mID, 0x24 );
    int highbyte = dxl_read_word( mID, 0x25 );
    int intval = dxl_makeword(lowbyte, highbyte);
    val = (double)intval; //translation?
    return val;
}


void Dynamixel::ProcessingThread()
{

}

bool Dynamixel::dxl_get_lowbyte(int word){return true;}
bool Dynamixel::dxl_get_highbyte(int word){return true;}
bool Dynamixel::dxl_makeword(int lowbyte, int highbyte){return true;}

/* End of File */
