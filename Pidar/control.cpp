////////////////////////////////////////////////////////////////////////////////
///
/// \file control.cpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include "control.hpp"
#include "global.hpp"
#include <sys/types.h>
#include <ifaddrs.h>

std::deque<pcl_data> gSendPoints;
bool gFoundUpdate = false;
bool gISRFlag = true;
bool gStopFlag = false;
int gMotorSpeed = 0;
unsigned short gLEDCount = 0;


using namespace Pidar;
using namespace PointCloud;


void InterruptServiceStop(void)
{
    gStopFlag = true;
    gMainControl->mMotor->SetSpeedRpm(0, true);
    sleep(1);
    gMainControl->mMotor->Shutdown();
}


void InterruptService(void)
{
    static Point3D::List previousList;
    static Point3D::List currentList;
    if(!gStopFlag)
    {
        // Flash activity (interrupt) light.
        if(gLEDCount % 3 == 0)
        {
            digitalWrite(LEDPIN, 1);
            gLEDCount = 0;
        }
        else
        {
            digitalWrite(LEDPIN, 0);
        }
        gLEDCount++;
        gISRFlag = true;

        // Get all necessary data for 3D data construction.
        if(gMainControl->mMotor->IsConnected())
        {
//            static bool first = true;
//            if(first)
//            {
//                currentList = gMainControl->mLaserScan;
//                previousList = currentList;
//                first = false;
//            }
//            previousList = currentList;
//            currentList = gMainControl->mLaserScan;
//            unsigned int waitCounter = 0;
//            if(previousList.size() > 0 && currentList.size() > 0)
//            {
//            while(fabs(previousList.front().GetX() -
//                    currentList.front().GetX()) < 0.0001 &&
//               fabs(previousList.back().GetY() -
//                    currentList.back().GetY()) < 0.0001)
//            {
//                currentList = gMainControl->mLaserScan;
//                waitCounter++;
//            }
//            }
//            if(waitCounter != 0)
//            {
//                std::cout << waitCounter << std::endl;
//            }
            gMainControl->AddToScanQueue(gMainControl->mLaserScan,
                                         gMainControl->mMotorAngle,
                                         gMainControl->mMotor->
                                            GetPreviousPositionDegrees());
            // Update previous motor position as current position.
            gMainControl->mMotor->
                    SetPreviousPositionDegrees(gMainControl->
                                               mMotor->
                                               GetCurrentPositionDegrees());
        }
        else
        {
            std::cout << "Motor is disconnected" << std::endl;
        }
    }
    else
    {
        digitalWrite(LEDPIN, 0);
    }
}


Control::Control()
{
    mMotor = new Motor::Dynamixel();
    mLaser = new Laser::Hokuyo();
}


Control::~Control()
{

}


bool Control::Initialize()
{
    //Test Switches
    bool enableLaser = true;
    bool enableMotor = true;
    bool enableISR = true;
    unsigned short restartCount = 0;

    if(enableLaser)
    {
        mLaser->RegisterCallback(this);
        while(!mLaser->Initialize())
        {
            restartCount++;
            sleep(1);
            if(restartCount > 3)
            {
                return false;
            }
        }
    }
    restartCount = 0;

    if(enableMotor)
    {
        mMotor->RegisterCallback(this);
        while(!mMotor->Initialize())
        {
            restartCount++;
            sleep(1);
            if(restartCount > 3)
            {
                return false;
            }
        }
        mMotor->SetSpeedRpm(1.0, true);
    }

    // Wait for movement/data
    sleep(1);

    if(enableISR)
    {
        if(wiringPiSetupSys () < 0)
        {
            fprintf (stderr,
                     "Unable to setup wiringPi: %s\n", strerror (errno));
        }
        // Pin 17/0 generate an interrupt on low-to-high transitions
        // and attach myInterrupt() to the interrupt
        if(wiringPiISR(HOKUYOSYNCPIN, INT_EDGE_RISING, InterruptService) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        if(wiringPiISR(SHUTDOWNPIN, INT_EDGE_RISING, InterruptServiceStop) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        // Setup LED pin mode
        pinMode(LEDPIN, OUTPUT);
    }
    return true;
}


void Control::AddToScanQueue(Point3D::List laserscan,
                             float currentMotorPosition,
                             float previousMotorPosition)
{
    int scancnt = laserscan.size();//1080;
    bool newScan = false;
    //Check for complete scan & get delta
    float delta_position = 0.0;
    if(previousMotorPosition < currentMotorPosition &&
       fabs(previousMotorPosition - currentMotorPosition) > 0.25)
    {
        // Motor has made a full revolution (360 degrees)
        delta_position = ((M_PI * 2.0 - previousMotorPosition)
                          - currentMotorPosition);
        newScan = true;
    }
    else
    {
        delta_position = previousMotorPosition - currentMotorPosition;
        if(currentMotorPosition <= M_PI && previousMotorPosition >= M_PI)
        {
            newScan = true;
        }
    }

    pcl_data tmp;
    if(laserscan.size() > 0)
    {
        for(int i = 0; i <= scancnt; i++)
        {
            pcl_point point;
            point.r = (laserscan[i]).GetX();
            point.theta = (laserscan[i]).GetY();
            point.phi = M_PI * 2.0
                        - float(previousMotorPosition +
                                (i * (delta_position / scancnt)));
            point.intensity = (laserscan[i]).GetIntensity();
            if(i == scancnt && newScan)
            {
                point.newScan = 1.0f;
            }
            else
            {
                point.newScan = 0.0f;
            }
            tmp.points.push_back(point);
        }
        //Add scan to queue
        gSendPoints.push_back(tmp);
    }
}


bool Control::StartISR()
{
    bool result = true;
    if (wiringPiSetupSys () < 0)
    {
        fprintf(stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        result = false;
    }
    // set Pin 17/0 generate an interrupt on low-to-high transitions
    // and attach myInterrupt() to the interrupt
    if (wiringPiISR(HOKUYOSYNCPIN, INT_EDGE_RISING, InterruptService) < 0)
    {
        fprintf(stderr, "Unable to setup ISR: %s\n", strerror (errno));
        result = false;
    }
    if (wiringPiISR(SHUTDOWNPIN, INT_EDGE_RISING, InterruptServiceStop) < 0)
    {
        fprintf(stderr, "Unable to setup ISR: %s\n", strerror (errno));
        result = false;
    }
    // Setup LED pin mode
    //system("gpio export 10 out");
    pinMode(LEDPIN, OUTPUT);
    return result;
}
/*End of File */
