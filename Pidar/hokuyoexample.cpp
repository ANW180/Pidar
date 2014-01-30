////////////////////////////////////////////////////////////////////////////////
///
/// \file hokuyoexample.cpp
/// \brief Example of using the hokuyo interface class.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <hokuyo.hpp>

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
/** End of File */
