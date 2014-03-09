////////////////////////////////////////////////////////////////////////////////
///
/// \file control.cpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include<control.hpp>

using namespace Pidar;
using namespace PointCloud;


#include<global.hpp>
void InterruptService(void)
{

    maincontrol->GetMotorPositionDegrees();


    //TODO: calculate / adjust values to put into the Data Structure
//    double current_position = Control::motor->GetPositionDegrees();
//    double previous_position;
//    double position_delta = previous_position - current_position;
//    std::vector<CvPoint3D64f> laserscan = Control::mpLasercallback.mLaserScan;

//    std::vector<pcl_data> singleScan;
//    int cnt = laserscan.size();
//    pcl_data temp;

//    for(int i = 0;i<540;i++)
//    {
//        temp.r[i] = laserscan[i].x; //not sure the structure here...?
//        temp.theta[i] = laserscan[i].y;
//        temp.phi[i] = previous_position+(position_delta/cnt)*(1+i); //Value in radians of motor
//    }

//    for(int i = 540;i<1082;i++)
//    {
//        temp.r[i] = laserscan[i].x; //not sure the structure here...?
//        temp.theta[i] = laserscan[i].y;
//        temp.phi[i] = previous_position+(position_delta/cnt)*(1+i); //Value in radians of motor
//    }
//    singleScan.push_back(temp);

    //TODO: Offload push and calculations to function
}




Control::Control()
{
    motor = new Motor::Dynamixel();
    laser = new Laser::Hokuyo();
}

Control::~Control()
{
    //Shutdown();
}






bool Control::Initialize()
{

    bool enableLaser = false;
    bool enableMotor = false;
    bool enableCOM = true;
    bool enableISR = false;

    if(enableLaser)
    {
        laser->RegisterCallback(&mpLasercallback);
        laser->Initialize();
    }

    if(enableMotor)
    {
        motor->RegisterCallback(&mpMotorcallback);
        motor->Initialize();
    }

    if(enableISR)
    {
        if (wiringPiSetup () < 0)
        {
            fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
            //   return 1;
        }

        // set Pin 17/0 generate an interrupt on low-to-high transitions
        // and attach myInterrupt() to the interrupt
        if ( wiringPiISR (HOKUYOSYNCPIN, INT_EDGE_RISING, InterruptService) < 0 )
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
            //   return 1;
        }

    }

    if(enableCOM)
    {

        try
        {
            unsigned short port = boost::lexical_cast<unsigned short>("10000");

            boost::asio::io_service io_service;

            PointCloud::server server(io_service, port);
            std::cout<<"Starting COM on port: " << port << std::endl;
            io_service.run();
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

    return true;

}

double Control::GetMotorPositionDegrees()
{
    return motor->GetPositionDegrees();
}

void Control::StartMotor(int rpm)
{
     Control::motor->SetSpeedRpm(rpm, true);
}

void Control::StopMotor()
{
     Control::motor->SetSpeedRpm(0, true);
     delay(1000);
     Control::motor->Shutdown();
}

void Control::StopLaser()
{
    Control::laser->Shutdown();
}

void Control::InterpolateScan(std::vector<CvPoint3D64f> scan, double startScanAngle,double stopScanAngle)
{
}
/* End of File */
