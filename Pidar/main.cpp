////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for testing different system classes
///        and functionality.
/// Author: Andrew Watson, Jonathan Ulrich
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com, JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef MAIN_CPP
#define MAIN_CPP
#include "hokuyo.hpp"
#include "dynamixel.hpp"
#include "control.hpp"
#include "global.hpp"
#include <wiringPi.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <ctime>
#include <iostream>
#include <string>


Pidar::Control* maincontrol;
//timespec t1;
int reset = 1;
using namespace std;

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


double MiddleScanDistanceInches (const std::vector<Point3D>& scan)
{
    if(scan.size() > 0)
    {
        std::vector<Point3D> copy = scan;
        return (copy.at(copy.size() / 2.).GetX()) * 1000. / 25.4;
    }
    else
    {
        return 0;
    }
}


class ExampleLaserCallback : public Laser::Callback
{
public:
    ExampleLaserCallback(){}
    ~ExampleLaserCallback(){}
    virtual void ProcessLaserData(const std::vector<Point3D>& scan,
                                  const time_t& timestamp)
    {
        mLaserScan = scan;
        //cout << MiddleScanDistanceInches(mLaserScan) << endl;
        cout << mLaserScan.at(mLaserScan.size()/2).GetX() << endl;
        mTimeStamp = timestamp;
    }
    std::vector<Point3D> mLaserScan;
    time_t mTimeStamp;
};


class ExampleDynamixelCallback : public Motor::Callback
{
public:
    ExampleDynamixelCallback(){}
    ~ExampleDynamixelCallback(){}
    virtual void ProcessServoData(const double& pos,
                                  const timespec& timestamp)
    {
        mMotorAngle = pos;
//        timespec t2 = timestamp;
//        //cout << mMotorAngle << endl;
//        timespec dif = diff(t1, t2);
//        cout << "Sec: "
//             << dif.tv_sec
//             << "Nsec: "
//             << dif.tv_nsec
//             << endl;
        mTimeStamp = timestamp;
        reset = 1;
    }
    double mMotorAngle;
    timespec mTimeStamp;
};


//#define TESTHOKUYO
#ifdef TESTHOKUYO
int main()
{
    Laser::Hokuyo* laser = new Laser::Hokuyo();
    ExampleLaserCallback callback;
    laser->RegisterCallback(&callback);
    if(laser->Initialize())
    {
        while (1)
        {
            boost::this_thread::sleep(
                        boost::posix_time::millisec(100));
        }
        laser->Shutdown();
        delete laser;
}

    return 0;
}
#endif


//#define TESTDYNAMIXELFEEDBACK
#ifdef TESTDYNAMIXELFEEDBACK
int main()
{
    Motor::Dynamixel* motor = new Motor::Dynamixel();
    ExampleDynamixelCallback callback;
    motor->RegisterCallback(&callback);
    double rpm = 0.0;
    if(motor->Initialize())
    {
        motor->SetSpeedRpm(rpm, true);
        clock_gettime(CLOCK_REALTIME, &t1);
        while (1)
        {
            if(reset == 1)
            {
                //std::cout << "I RESET" << std::endl;
                clock_gettime(CLOCK_REALTIME, &t1);
                reset = 0;
            }
            //std::cout << "I RESET" << std::endl;
//            timespec sleep, remaining;
//            sleep.tv_sec = remaining.tv_sec = 0;
//            sleep.tv_nsec = 1000L; //25 microseconds
//            nanosleep(&sleep, &remaining);
        }
    }
   return 0;
}
#endif


//#define TESTMOTORLASER
#ifdef TESTMOTORLASER
int main()
{
    Laser::Hokuyo* laser = new Laser::Hokuyo();
    ExampleLaserCallback lcallback;
    laser->RegisterCallback(&lcallback);
    Motor::Dynamixel* motor = new Motor::Dynamixel();
    ExampleDynamixelCallback mcallback;
    motor->RegisterCallback(&mcallback);
    if(motor->Initialize())
    {
        motor->SetSpeedRpm(0, true);
        if(laser->Initialize())
        {
            while(1)
            {
                boost::this_thread::sleep(
                            boost::posix_time::millisec(10000000));
            }
        }
    }
}
#endif


//#define TESTDYNAMIXELSPEED
#ifdef TESTDYNAMIXELSPEED

int main()
{
    Motor::Dynamixel* motor = new Motor::Dynamixel();
    double rpm = 0.0;
    int dir;
    bool clockwise = true;
    if(motor->Initialize())
    {
        motor->SetSpeedRpm(rpm, clockwise);
        while (1)
        {
            cout << "Input Speed in RPM (0-114): " << endl;
            cin >> rpm;
            cout << "Input direction (0 = CCW, 1 = CW): " << endl;
            cin >> dir;
            if(dir)
                clockwise = true;
            else
                clockwise = false;
            motor->SetSpeedRpm(rpm, clockwise);
        }
    }
   return 0;
}
#endif


/** This test tells the motor to move at 10 RPM and sets a timer at the
    start of the sequence, upon 1 revolution it will print the time
    elapsed. Theoretically it should be 6 seconds exactly given 10 RPM.*/
//#define TESTDYNAMIXELSPEEDACCURACY
#ifdef TESTDYNAMIXELSPEEDACCURACY

int main()
{
    Motor::Dynamixel* motor = new Motor::Dynamixel();
    double rpm = 10.0; // 10 RPM / 60 = .16667 Hz ^ -1 = 6 secs
    double degrees = 0;
    double lowthres = 0;
    double hghthres = 0;
    double difseconds = 0;
    double curpos = 0;
    timespec t1, t2, dif;
    if(motor->Initialize())
    {
        delayMicroseconds(250000);
        motor->SetSpeedRpm(0, true);
        delayMicroseconds(750000);
        degrees = motor->GetPositionDegrees();
        lowthres = degrees - 0.3;
        hghthres = degrees + 0.3;
        motor->SetSpeedRpm(rpm, true);
        delayMicroseconds(40000);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        while (1)
        {
            curpos = motor->GetPositionDegrees();
            if(curpos >= lowthres && curpos <= hghthres)
            {
                clock_gettime(CLOCK_MONOTONIC, &t2);
                dif = diff(t1, t2);
                double decimalsecs = (double) dif.tv_nsec;
                decimalsecs /= 1000000000.;
                difseconds = dif.tv_sec + decimalsecs;
                cout << setprecision(7) << difseconds << endl;
                motor->SetSpeedRpm(0, true);
                delayMicroseconds(250000);
                break;
            }
        }
    }
   return 0;
}
#endif


//#define TESTTIMING
#ifdef TESTTIMING
int main()
{
    timespec t1, t2;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
    unsigned int i = 0;
    while(1)
    {
        if(diff(t1, t2).tv_sec >= 1)
        {
            i++;
            std::cout << diff(t1, t2).tv_nsec <<" Sec Elapsed" << std::endl;
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
        }
        else
        {
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
        }
    }
    return 0;
}

#endif


//#define TESTSERIALSERVER
#ifdef TESTSERIALSERVER
#include <connection.cpp>

int main(int argc, char* argv[])
{
  try
  {
    // Check command line arguments.
    if (argc != 2)
    {
      //std::cerr << "Usage: server <port>" << std::endl;
      //return 1;
      std::cerr << "Defaulting to Port 10000" << std::endl;
      argv[1]="10000";
    }
    unsigned short port = boost::lexical_cast<unsigned short>(argv[1]);

    boost::asio::io_service io_service;

    pointcloud_connection::server server(io_service, port);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

#endif

//#define TESTISR
#ifdef TESTISR

#define BUTTON_PIN 17
volatile int eventCounter = 0;
void myInterrupt(void)
{
    eventCounter++;
}

int main()
{
    if (wiringPiSetupSys() < 0)
    {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        return 1;
    }
    // set Pin 17/0 generate an interrupt on high-to-low transitions
    // and attach myInterrupt() to the interrupt
    if ( wiringPiISR (BUTTON_PIN, INT_EDGE_FALLING, &myInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }
    // display counter value every second.
    while (1)
    {
        std::cout << eventCounter << std::endl;
        eventCounter = 0;
        delay(1000); // wait 1 second
    }
      return 0;
}

#endif


#define TESTIMAGEGENERATION
#ifdef TESTIMAGEGENERATION
pcl_data getRandomData()
{
    pcl_data d;
    for(int i = 0;i<1081;i++)
    {
        pcl_point x;
        x.r = (float(rand()%1000))/1000;
        x.theta = (float(rand()%50))/10;
        x.phi = (float(rand()%360));
        d.points.push_back(x);

    }
    return d;
}

int main()
{
    maincontrol = new Pidar::Control();
    maincontrol->Initialize();
    globMotorSpeed = 10;
    int restartCnt = 0;
    maincontrol->StartMotor(globMotorSpeed);
    sleep(1);

    //Start Web Server Thread
    boost::asio::io_service io_service;
    PointCloud::server s(io_service, boost::asio::ip::address::from_string("239.255.0.1"));
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service));

    //Start Seperate Command Web Server
    boost::asio::io_service io_service_cmds;
    commandserver cmdsrv(io_service_cmds,10001);
    boost::thread bt2(boost::bind(&boost::asio::io_service::run, &io_service_cmds));


    while(1)
    {
        if(ISRrunning)
        {
            //std::cout<<"ISR restarted "<< restartCnt << " times" << std::endl;
            if(restartCnt > 0)
            {
                std::cout << "ISR restarted " << restartCnt << " times" << std::endl;
            }
            ISRrunning = false;
        }
        else
        {
            std::cout << "ISR NOT running, restarting " << restartCnt
                      << " restart attempts made." << std::endl;
            maincontrol->StartISR();
            boost::this_thread::sleep(
                        boost::posix_time::millisec(1000));
            restartCnt++;
        }
        //Check for updates from client
        if(globFoundUpdate)
        {
            maincontrol->StartMotor(globMotorSpeed);
            globFoundUpdate = false;

            //If stopping motor, stop whole program
            if(globMotorSpeed == 0)
                return 0;
        }
        // Fix for laser disconnecting on slipring?
        // Shutdown thread and serial connection, sleep, then
        // restart both to resume data output.
        if(maincontrol->GetLaserPtr()->GetErrorCount() > 3)
        {
            std::cout << "Restarting Laser Connection" << std::endl;
            maincontrol->GetLaserPtr()->Shutdown();
            sleep(2);
            maincontrol->GetLaserPtr()->Initialize();
            sleep(2);
        }
        //std::cout<<"Current Speed: "<<globMotorSpeed<<std::endl;
        //std::cout << "Buffer Size: " << SendPoints.size() << std::endl;
        sleep(1);
    }
   return 0;
}
#endif

#endif

/** End of File */
