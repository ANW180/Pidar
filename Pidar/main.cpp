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
#include "hokuyo.hpp"
#include "dynamixel.hpp"
#include <time.h>
#include <wiringPi.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
using namespace std;
timespec diff(timespec start, timespec end);
//#define TESTHOKUYO
#ifdef TESTHOKUYO
class ExampleLaserCallback : public Laser::Callback
{
public:
    ExampleLaserCallback(){}
    ~ExampleLaserCallback(){}
    virtual void ProcessLaserData(const std::vector<CvPoint3D64f>& scan,
                                  const time_t& timestamp)
    {
        mLaserScan = scan;
        mTimeStamp = timestamp;
    }
    std::vector<CvPoint3D64f> mLaserScan;
    time_t mTimeStamp;
};

double MiddleScanDistanceInches (const std::vector<CvPoint3D64f>& scan)
{
    if(scan.size() > 0)
        return scan.at(scan.size() / 2.).x * 1000. / 25.4;
    else
        return 0;
}

int main()
{
    Laser::Hokuyo* laser = new Laser::Hokuyo();
    ExampleLaserCallback callback;
    laser->RegisterCallback(&callback);
    if(laser->Initialize())
    {
        if(laser->StartCaptureThread())
        {
            while (1)
            {
                std::cout << MiddleScanDistanceInches(callback.mLaserScan)
                          << std::endl;
                boost::this_thread::sleep(boost::posix_time::millisec(10));
            }
            laser->Shutdown();
            delete laser;
        }
    }

    return 0;
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


//#define TESTDYNAMIXELFEEDBACK
#ifdef TESTDYNAMIXELFEEDBACK

int main()
{
    Motor::Dynamixel* motor = new Motor::Dynamixel();
    double rpm = 10.0;
    if(motor->Initialize())
    {
        motor->SetSpeedRpm(rpm, true);
        while (1)
        {
            cout << motor->GetPositionDegrees() << endl;
            usleep(10000);
        }
    }
   return 0;
}
#endif


/** This test tells the motor to move at 10 RPM and sets a timer at the
    start of the sequence, upon 1 revolution it will print the time
    elapsed. Theoretically it should be 6 seconds exactly given 10 RPM.*/
#define TESTDYNAMIXELSPEEDACCURACY
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





//#define TESTSERVER
#ifdef TESTSERVER
//http://www.boost.org/doc/libs/1_45_0/doc/html/boost_asio/example/iostreams/
//Requires SUDO to run in order to bind to port
using boost::asio::ip::tcp;
std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

int main()
{
  try
  {
    boost::asio::io_service io_service;

    tcp::endpoint endpoint(tcp::v4(), 13);
    tcp::acceptor acceptor(io_service, endpoint);

    for (;;)
    {
      tcp::iostream stream;
      acceptor.accept(*stream.rdbuf());
      stream << make_daytime_string();
    }
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

#define BUTTON_PIN 0
volatile int eventCounter = 0;
void myInterrupt(void) {eventCounter++;}

int main()
{

    if (wiringPiSetup () < 0) {
          fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
          return 1;
      }

      // set Pin 17/0 generate an interrupt on high-to-low transitions
      // and attach myInterrupt() to the interrupt
      if ( wiringPiISR (BUTTON_PIN, INT_EDGE_FALLING, &myInterrupt) < 0 ) {
          fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
          return 1;
      }

      // display counter value every second.
      while ( 1 ) {
        printf( "%d\n", eventCounter );
        eventCounter = 0;
        delay( 1000 ); // wait 1 second
      }

      return 0;
}

#endif

//#define TESTIMAGEGENERATION
#ifdef TESTIMAGEGENERATION
int main()
{

   return 0;
}
#endif

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if((end.tv_nsec - start.tv_nsec) < 0)
    {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000L + end.tv_nsec - start.tv_nsec;
    }
    else
    {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

/** End of File */
