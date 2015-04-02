/**
  \file Control.cpp
  \brief Class for managing control of the Pidar components.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Control.hpp"
#include "Global.hpp"
#include <sys/types.h>
#include <ifaddrs.h>

using namespace Pidar;


Control::Control()
{
    mpMotor = new Motor::Dynamixel();
    mpLaser = new Laser::Hokuyo();
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
        mpLaser->RegisterCallback(this);
        while(!mpLaser->Initialize())
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
        mpMotor->RegisterCallback(this);
        while(!mpMotor->Initialize())
        {
            restartCount++;
            sleep(1);
            if(restartCount > 3)
            {
                return false;
            }
        }
        mpMotor->SetSpeedRpm(1.0, true);
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
        if(wiringPiISR(HOKUYOSYNCPIN,
                       INT_EDGE_RISING,
                       InterruptService) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        if(wiringPiISR(SHUTDOWNBUTTONPIN,
                       INT_EDGE_RISING,
                       InterruptServiceStop) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        // Setup LED pin mode
        pinMode(LEDPIN, OUTPUT);
    }
    return true;
}


bool Control::SetupWiringPi()
{
    bool result = true;
    if(wiringPiSetupSys () < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup wiringPi: "<< strerror(errno) << std::endl;
#endif
        result = false;
    }
    // set Pin 17/0 generate an interrupt on low-to-high transitions
    // and attach myInterrupt() to the interrupt
    if(wiringPiISR(HOKUYOSYNCPIN,
                   INT_EDGE_RISING,
                   InterruptService) < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup ISR: "<< strerror(errno) << std::endl;
#endif
        result = false;
    }
    if(wiringPiISR(SHUTDOWNBUTTONPIN,
                   INT_EDGE_RISING,
                   InterruptServiceStop) < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup ISR: "<< strerror(errno) << std::endl;
#endif
        result = false;
    }
    // Setup LED pin mode
    pinMode(LEDPIN, OUTPUT);
    return result;
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


void Control::ProcessLaserData(const Point3D::List& polarScan,
                               const timespec& timestampUTC)
{
    mLaserScan = polarScan;
    mLaserTimestamp = timestampUTC;
}


void Control::ProcessServoData(const float &positionRadians,
                               const timespec &timestampUTC)
{
    mMotorAngle = positionRadians;
    mMotorTimestamp = timestampUTC;
}


void InterruptServiceStop(void)
{
    gStopFlag = true;
    gMainControl->mpMotor->SetSpeedRpm(0.0, true);
    sleep(1);
    gMainControl->mpMotor->Shutdown();
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
        if(gMainControl->mpMotor->IsConnected())
        {
            //            static bool first = true;
            //            if(first)
            //            {
            //                currentList = gMainControl->mpLaserScan;
            //                previousList = currentList;
            //                first = false;
            //            }
            //            previousList = currentList;
            //            currentList = gMainControl->mpLaserScan;
            //            unsigned int waitCounter = 0;
            //            if(previousList.size() > 0 && currentList.size() > 0)
            //            {
            //            while(fabs(previousList.front().GetX() -
            //                    currentList.front().GetX()) < 0.0001 &&
            //               fabs(previousList.back().GetY() -
            //                    currentList.back().GetY()) < 0.0001)
            //            {
            //                currentList = gMainControl->mpLaserScan;
            //                waitCounter++;
            //            }
            //            }
            //            if(waitCounter != 0)
            //            {
            //                std::cout << waitCounter << std::endl;
            //            }
            gMainControl->AddToScanQueue(gMainControl->mpLaserScan,
                                         gMainControl->mpMotorAngle,
                                         gMainControl->mpMotor->
                                         GetPreviousPositionDegrees());
            // Update previous motor position as current position.
            gMainControl->mpMotor->
                    SetPreviousPositionDegrees(gMainControl->
                                               mpMotor->
                                               GetCurrentPositionDegrees());
        }
        else
        {
#ifdef DEBUG
            std::cout << "Motor is disconnected" << std::endl;
#endif
        }
    }
    else
    {
        digitalWrite(LEDPIN, 0);
    }
}
/*End of File */
