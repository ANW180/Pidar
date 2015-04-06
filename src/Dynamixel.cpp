/**
  \file Dynamixel.cpp
  \brief Interface for connecting to Dynamixel Motors. Wraps Dynamixel DXL SDK
        available here: http://support.robotis.com/en/software/dynamixelsdk.html
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Dynamixel.hpp"
#include "Global.hpp"

using namespace Pidar;
using namespace Motor;

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
//    if(s > 1.0 && tme.size() > 0)
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


Dynamixel::Dynamixel()
{
    mConnectedFlag = false;
    mID = 1;
    mSerialPort = "/dev/ttyUSB0";
    mCommandSpeedRpm = mCurrentPositionRad = 0.0;
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
#ifdef DEBUG
    std::cout << "Connected to Dynamixel Successfully" << std::endl;
#endif
    //clock_gettime(CLOCK_MONOTONIC, &t3);
    return true;
}


void Dynamixel::Shutdown()
{
    if(IsConnected())
    {
        dxl_terminate();
        mConnectedFlag = false;
    }
}


bool Dynamixel::IsConnected()
{
    return mConnectedFlag;
}


void Dynamixel::SetSpeedRpm(const float rpm)
{
    // Threshold speed to within max allowed speed.
    mCommandSpeedRpm = rpm;
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
    // Add 1024 to offset for desired direction (clockwise).
    val += 1024.0;
    int CommStatus = COMM_RXFAIL;
    int loopCount = 0;
    while(CommStatus != COMM_RXSUCCESS)
    {
        dxl_write_word(mID, MOVING_SPEED_L, val);
        CommStatus = dxl_get_result();
        loopCount++;
    }
    if(loopCount > 1)
    {
#ifdef DEBUG
        std::cout << "Dynamixel Write Cnt: "
                  << loopCount
                  << std::endl;
#endif
    }
}


// Average read delay time is 1 - 1.5 ms.
float Dynamixel::GetCurrentPositionRadians()
{
    //clock_gettime(CLOCK_MONOTONIC, &t1);
    int CommStatus = COMM_RXFAIL;
    int recv;
    int loopCount = 0;
    while(CommStatus != COMM_RXSUCCESS)
    {
        recv = dxl_read_word(mID, PRESENT_POSITION_L);
        CommStatus = dxl_get_result();
        loopCount++;
    }
    if(loopCount > 1)
    {
#ifdef DEBUG
        std::cout << "Dynamixel Read Cnt: "
                  << loopCount
                  << std::endl;
#endif
    }
    mCurrentPositionRad = recv * MX28_RAD_PER_UNIT;
    //computeAvg();
    return mCurrentPositionRad;
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
