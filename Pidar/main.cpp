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
#include <hokuyo.hpp>
#include <dynamixel.hpp>
#include <time.h>
#include <wiringPi.h> //wiringpi testing

//Webserver testing
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
//end webserver testing

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


#define TESTDYNAMIXEL
#ifdef TESTDYNAMIXEL

int main()
{

    Motor::Dynamixel* motor = new Motor::Dynamixel();
    double rpm = 10;

    if(motor->Initialize())
    {
        motor->SetSpeedRpm(rpm);
        //sleep(1);
        while (1)
        {
            std::cout << motor->GetPositionPercent() << std::endl;
            //sleep(1);
        }
    }

   return 0;
}
#endif



//#define TESTTIMING
#ifdef TESTTIMING
timespec diff(timespec start, timespec end);
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
timespec diff(timespec start, timespec end)
{
    timespec temp;
    if((end.tv_nsec-start.tv_nsec)<0)
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

/** End of File */
