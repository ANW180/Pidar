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
    return 0;
}
#endif



#define TESTTIMING
#ifdef TESTTIMING
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
    timespec ts;
    long prevtime;
    long currtime;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    prevtime = currtime = ts.tv_nsec;
    while (1)
    {
        if (currtime - prevtime >= 100000000)
        {
            std::cout << "1 Second Passed" << std::endl;
            prevtime = currtime;
        }
        clock_gettime(CLOCK_MONOTONIC, &ts);
        std::cout << ctime(&ts.tv_nsec) << std::endl;
    }
    return 0;
}
#endif





//#define TESTSERVER
#ifdef TESTSERVER
int main()
{
    return 0;
}

#endif
/** End of File */
