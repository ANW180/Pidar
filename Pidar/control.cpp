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
pcl_data PublicScan;


#include <fstream>
std::ifstream infile("../Pidar/sampledata.txt");
pcl_data GetSampleTXTFileData(){
    pcl_data Scan;
    double a, b, c = 0.0;
    std::string str;
    int linecnt = 0;
    std::string delimiter = ",";
    std::string val;
     while (std::getline(infile, str))
    {
         int x = 0;
         size_t pos = 0;
         while ((pos = str.find(delimiter)) != std::string::npos) {
             val = str.substr(0, pos);

             if(x==0){
                 a = atof(val.c_str());x++;
             }
             else if(x==1){
                 b = atof(val.c_str());x++;
             }
             else if(x==2){
                 c = atof(val.c_str());x=0;
             }
             str.erase(0, pos + delimiter.length());
         }

        pcl_point point;
        point.r = a;
        point.theta = b;
        point.phi = c;
        Scan.points.push_back(point);
        linecnt++;
    }
    linecnt;
    return Scan;
}

#include<global.hpp>
void InterruptService(void)
{

    double current_position = maincontrol->GetMotorPositionDegrees();
    double previous_position = maincontrol->GetMotorPreviousPositionDegrees();

    //Update Previous Motor Position
    maincontrol->SetMotorPreviousPositionDegrees(current_position);

    //Get Values to send for construction
    std::vector<CvPoint3D64f> laserscan = maincontrol->GetLaserScan();
    pcl_data Incomplete = maincontrol->getIncompleteConstruction();

    //Send values and get newly constructed incompletescan
    Incomplete = maincontrol->addtoScanConstruction(Incomplete, laserscan,
                                      current_position,previous_position);


    //return the appended incomplete scan
    maincontrol->setIncompleteConstruction(Incomplete);



    //TODO: Offload push and calculations to function
}




Control::Control()
{
    motor = new Motor::Dynamixel();
    laser = new Laser::Hokuyo();
    construct = new PointCloud::Construction();
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

    ///******* TESTING ONLY **********************************
    //This is for testing only (remove when not testing)
    PublicScan = GetSampleTXTFileData();
    PublicScan.id = 1; //initialization
    PublicScan.speed = 88;
    std::cout<<"control.cpp: ID: "<<PublicScan.id<<std::endl;
    std::cout<<"control.cpp: r: "<<PublicScan.points[0].r<<std::endl;
    std::cout<<"control.cpp: theta: "<<PublicScan.points[0].theta<<std::endl;
    std::cout<<"control.cpp: phi: "<<PublicScan.points[0].phi<<std::endl;
    ///******** END TESTING LINES ****************************


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

double Control::GetMotorPreviousPositionDegrees(){
    double val = motor->GetPreviousPositionDegrees();
}

void Control::SetMotorPreviousPositionDegrees(double val){
   motor->SetPreviousPositionDegrees(val);
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

std::vector<CvPoint3D64f> Control::GetLaserScan(){
    return Control::mpLasercallback.mLaserScan;
}

pcl_data Control::getIncompleteConstruction(){
        return construct->getIncompleteScan();
}

pcl_data Control::getCompleteConstruction(){
        return construct->getCompleteScan();
}


Pidar::Control* getMainControl(){
   return maincontrol;
}


void Control::setIncompleteConstruction(pcl_data data){
    construct->setIncompleteScan(data);
}

pcl_data Control::addtoScanConstruction(pcl_data Incomplete, std::vector<CvPoint3D64f> laserscan,
                                 double currentMotorPosition, double previousMotorPosition){

    return construct->addtoScan(Incomplete, laserscan,
                                currentMotorPosition,previousMotorPosition);

}

void Control::InterpolateScan(std::vector<CvPoint3D64f> scan, double startScanAngle,double stopScanAngle)
{
}
/* End of File */
