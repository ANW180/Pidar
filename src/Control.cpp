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
#include <time.h>

using namespace Pidar;


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


Control::Control()
{
    mpMotor = new Motor::Dynamixel();
    mpLaser = new Laser::Hokuyo();
}


bool Control::Initialize()
{
    unsigned short restartCount = 0;
    if(!Control::Instance()->SetupWiringPi())
    {
        return false;
    }
    while(!mpMotor->Initialize())
    {
        restartCount++;
        sleep(1);
        if(restartCount > 3)
        {
            return false;
        }
    }
    mpMotor->SetSpeedRpm(0.0);
    restartCount = 0;
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
    return true;
}


bool Control::SetupWiringPi()
{
    bool result = true;
    // set Pin 17/0 generate an interrupt on low-to-high transitions
    // and attach myInterrupt() to the interrupt
    if(wiringPiISR(HOKUYO_SYNC_PIN,
                   INT_EDGE_RISING,
                   InterruptService) < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup ISR: "<< strerror(errno) << std::endl;
#endif
        result = false;
    }
    if(wiringPiISR(SHUTDOWN_BUTTON_PIN,
                   INT_EDGE_RISING,
                   InterruptServiceStop) < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup ISR: "<< strerror(errno) << std::endl;
#endif
        result = false;
    }
    // Setup LED pin mode
    pinMode(LED_PIN, OUTPUT);
    //clock_gettime(CLOCK_MONOTONIC, &t3);
    return result;
}


void Control::AddToScanQueue(Point3D::List laserscan,
                             float currentMotorPosition,
                             float previousMotorPosition)
{
    int scanCount = laserscan.size(); // 1080 steps
    bool newScan = false;
    float deltaPosition = 0.0;
    // Check for complete scan & get delta
    if(previousMotorPosition < currentMotorPosition &&
            fabs(previousMotorPosition - currentMotorPosition) > 0.25)
    {
        // Motor has made a full revolution (360 degrees)
        deltaPosition = ((M_PI * 2.0 - previousMotorPosition)
                         - currentMotorPosition);
        newScan = true;
    }
    else
    {
        deltaPosition = previousMotorPosition - currentMotorPosition;
        if(currentMotorPosition <= M_PI && previousMotorPosition >= M_PI)
        {
            newScan = true;
        }
    }

    pcl_data tmp;
    if(laserscan.size() > 0)
    {
        float changePerStep = deltaPosition / scanCount;
        if(changePerStep < 0.)
        {
#ifdef DEBUG
            std::cout << "Platform moving backwards!" << std::endl;
#endif
        }
        for(int i = 0; i <= scanCount; i++)
        {
            pcl_point point;
            point.r = (laserscan[i]).GetX();
            point.theta = (laserscan[i]).GetY();
            point.phi = M_PI * 2.0 - float(previousMotorPosition +
                                           (i * (changePerStep)));
            point.intensity = (laserscan[i]).GetIntensity();
            if(i == scanCount && newScan)
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
                               const long timestamp)
{
    mLaserScan = polarScan;
    mLaserTimestamp = timestamp;
}


Control* Control::Instance()
{
    if(!mpInstance)
    {
        mpInstance = new Control;
    }
    return mpInstance;
}


// Called every 20 - 25 ms on average.
void Pidar::InterruptService(void)
{
    if(!gStopFlag)
    {
        // Flash activity (interrupt) light.
        if(gLEDCount % 3 == 0)
        {
            digitalWrite(LED_PIN, 1);
            gLEDCount = 0;
        }
        else
        {
            digitalWrite(LED_PIN, 0);
        }
        gLEDCount++;
        gISRFlag = true;

        // Get all necessary data for 3D data construction if devices are ready.
        if(Control::Instance()->mpMotor->IsConnected() &&
                Control::Instance()->mpLaser->IsConnected())
        {
            float currMotorPosition =
                    Control::Instance()->mpMotor->GetCurrentPositionRadians();
            //clock_gettime(CLOCK_MONOTONIC, &t1);
            //computeAvg();
            //clock_gettime(CLOCK_MONOTONIC, &t1);
            Control::Instance()->AddToScanQueue(Control::Instance()->mLaserScan,
                                                Control::Instance()->mMotorAngle,
                                                Control::Instance()->mMotorAngle);
            // Update previous motor position as current position.
            //            Control::Instance()->mpMotor->
            //                    SetPreviousPositionRadians(Control::Instance()->
            //                                               mpMotor->
            //                                               GetCurrentPositionRadians());
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
        digitalWrite(LED_PIN, 0);
    }
}


void Pidar::InterruptServiceStop(void)
{
    gStopFlag = true;
    Control::Instance()->mpMotor->SetSpeedRpm(0.0);
}
/*End of File */
