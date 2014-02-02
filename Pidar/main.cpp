////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for testing different system classes
///        and functionality.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <hokuyo.hpp>
#include <time.h>
#include <wiringPi.h>

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


//#define TESTDYNAMIXEL
#ifdef TESTDYNAMIXEL
int main()
{
    int x = 1234;

    char bytes[sizeof x];
    std::copy(static_cast<const char*>(static_cast<const void*>(&x)),
              static_cast<const char*>(static_cast<const void*>(&x)) + sizeof x,
              bytes);

    char a = bytes[0];
    char b = bytes[1];




   return 0;
}
#endif



//#define TESTTIMING
#ifdef TESTTIMING
timespec diff(timespec start, timespec end);
int main()
{
//    clockid_t types[] = {CLOCK_REALTIME, CLOCK_MONOTONIC,
//                         CLOCK_PROCESS_CPUTIME_ID, CLOCK_THREAD_CPUTIME_ID,
//                         (clockid_t) - 1};
//    struct timespec spec;
//    for (int i = 0; types[i] != (clockid_t) - 1; i++)
//    {
//        if(clock_getres(types[i], &spec) != 0)
//        {
//            std::cout << "Timer " << types[i] << " not supported" << std::endl;
//        }
//        else
//        {
//            std::cout << "Timer: " << i << " Seconds: " << spec.tv_sec
//                      << " Nanos: " << spec.tv_nsec << std::endl;
//        }
//    }
    timespec t1, t2;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
    while(1)
    {
        if(diff(t1, t2).tv_nsec > 2000000)
        {
            std::cout << "1 Sec Elapsed" << std::endl;
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
        }
        else
        {
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
        }
        std::cout << diff(t1, t2).tv_nsec << std::endl;
        nanosleep((struct timespec[]){{0, 100000000}}, NULL);
    }
    return 0;
}
timespec diff(timespec start, timespec end)
{
    timespec temp;
    if((end.tv_nsec-start.tv_nsec)<0)
    {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
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
int main()
{
    return 0;
}

#endif



#define TESTISR
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
/** End of File */
