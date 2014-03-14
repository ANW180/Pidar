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
pcl_data PublicPartialScan;


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
double current_position;
double previous_position;
std::vector<CvPoint3D64f> laserscan;
void InterruptService(void)
{

    //Get Current and Previous Positions
    current_position = maincontrol->GetMotorPositionDegrees();
    previous_position = maincontrol->GetMotorPreviousPositionDegrees();

    //Update Previous Motor Position
    maincontrol->SetMotorPreviousPositionDegrees(current_position);

    //Get Values to send for construction
    laserscan = maincontrol->GetLaserScan();

    //Send values and get newly constructed incompletescan
    //this will update values as needed
    maincontrol->addtoScanConstruction(laserscan,
                                      current_position,previous_position);
}


///
/// Controller Functions
///
Control::Control()
{
    motor = new Motor::Dynamixel();
    laser = new Laser::Hokuyo();
    PublicPartialScan.scancount = 0;
}

Control::~Control()
{
    //Shutdown();
}

bool Control::Initialize()
{

    //Test Switches
    bool enableLaser = true;
    bool enableMotor = true;
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


    ///
    ///  Motor Functions
    ///
    double Control::GetMotorPositionDegrees()
    {
        return motor->GetPositionDegrees();
    }

    double Control::GetMotorPreviousPositionDegrees(){
        return motor->GetPreviousPositionDegrees();
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

    ///
    /// Laser Functions
    ///
    void Control::StopLaser()
    {
        Control::laser->Shutdown();
    }

    std::vector<CvPoint3D64f> Control::GetLaserScan(){
    return Control::mpLasercallback.mLaserScan;
}

    ///
    /// Scan Management Functions
    ///
    pcl_data Control::getIncompleteScan(){
              return PublicPartialScan;
    }

    pcl_data Control::getCompleteScan(){
            return PublicScan;
    }

    void Control::setCompleteScan(pcl_data data){
        PublicScan = data;
    }

    void Control::setIncompleteConstruction(pcl_data data){
        PublicPartialScan = data;
    }

    void Control::clearIncompleteScan(){
        PublicPartialScan.id = 0;
        PublicPartialScan.message = "";
        PublicPartialScan.points.clear();
        PublicPartialScan.scancount = 0;
    }

    void Control::addtoScanConstruction(std::vector<CvPoint3D64f> laserscan,
                                     double currentMotorPosition, double previousMotorPosition){

        int scancnt = laserscan.size();//1080;

        //Check for complete scan & get delta
        double delta_position = 0.0;
        bool scancomplete = false;
        if(currentMotorPosition<previousMotorPosition){
            delta_position = ((360-previousMotorPosition)+currentMotorPosition);
            scancomplete = true;
        }
        else
        {
            delta_position = (currentMotorPosition-previousMotorPosition);
            scancomplete = false;
        }

        for(int i = 0;i<scancnt;i++)
        {
                pcl_point point;
                point.r = laserscan[i].x;
                point.theta = laserscan[i].y;
                point.phi = previousMotorPosition + (i*(delta_position/scancnt));
                PublicPartialScan.points.push_back(point);
        }

        PublicPartialScan.scancount++;

        // If complete set the complete scan and copy to globally
        // accessible object
        if(scancomplete)
    {
        setCompleteScan(PublicPartialScan);
        clearIncompleteScan();
    }
}


/*End of File */
