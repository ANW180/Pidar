////////////////////////////////////////////////////////////////////////////////
///
/// \file control.hpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <hokuyo.hpp>
#include <pointcloud.hpp>
#include <dynamixel.hpp>
#include <wiringPi.h>
#include <connection.cpp>

#define HOKUYOSYNCPIN 0

namespace Pidar
{
    class LaserCallback : public Laser::Callback
    {
    public:
        LaserCallback(){}
        ~LaserCallback(){}
        virtual void ProcessLaserData(const std::vector<CvPoint3D64f>& scan,
                                      const time_t& timestamp)
        {
            mLaserScan = scan;
            mTimeStamp = timestamp;
            count++;
        }
        unsigned int count;
        std::vector<CvPoint3D64f> mLaserScan;
        time_t mTimeStamp;
    };

    class DynamixelCallback : public Motor::Callback
    {
    public:
        DynamixelCallback(){}
        ~DynamixelCallback(){}
        virtual void ProcessServoData(const double& pos,
                                      const timespec& timestamp)
        {
            mMotorAngle = pos;
            mTimeStamp = timestamp;
        }
        double mMotorAngle;
        timespec mTimeStamp;
    };


    class Control
    {
    public:
        Control();
        ~Control();


        bool Initialize(); //laser, motor,

        void InterpolateScan(std::vector<CvPoint3D64f> scan,
                                     double startScanAngle,
                                     double stopScanAngle);

        double GetMotorPositionDegrees();

        double GetMotorPreviousPositionDegrees();

        void SetMotorPreviousPositionDegrees(double val);

        double GetLaserPositionPolar();

        void StartMotor(int rpm);

        void StopMotor();

        void StopLaser();

        PointCloud::pcl_data getIncompleteConstruction();

        void setIncompleteConstruction(PointCloud::pcl_data data);

        PointCloud::pcl_data addtoScanConstruction(PointCloud::pcl_data Incomplete, std::vector<CvPoint3D64f> laserscan,
                                       double currentMotorPosition, double previousMotorPosition);

        std::vector<CvPoint3D64f> GetLaserScan();

        //static Control* GetInstance(){return maincontrol;}


    protected:
       // static Control *maincontrol;
        Motor::Dynamixel* motor;
        Laser::Hokuyo *laser;
        PointCloud::Construction *construct;
        std::vector<CvPoint3D64f> mScan;
        time_t mLaserTimestamp;
        time_t mMotorTimestamp;
        double mStartScanAngle;
        double mStopScanAngle;

        DynamixelCallback mpMotorcallback;
        LaserCallback mpLasercallback;
        PointCloud::pcl_data mPointCloud;
    };



}

#endif
