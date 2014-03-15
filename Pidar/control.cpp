////////////////////////////////////////////////////////////////////////////////
///
/// \file control.cpp
/// \brief Control the Pidar
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include "control.hpp"
#include "global.hpp"
double current_position;
double previous_position;
std::vector<Point3D> laserscan;
pcl_data PublicScan;
pcl_data PublicPartialScan;

using namespace Pidar;
using namespace PointCloud;


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
    maincontrol->AddToScanConstruction(laserscan,
                                      current_position,previous_position);
}


std::ifstream infile("../Pidar/sampledata.txt");
pcl_data GetSampleTXTFileData()
{
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
         while ((pos = str.find(delimiter)) != std::string::npos)
         {
             val = str.substr(0, pos);
             if(x == 0)
             {
                 a = atof(val.c_str());
                 x++;
             }
             else if(x == 1)
             {
                 b = atof(val.c_str());
                 x++;
             }
             else if(x == 2)
             {
                 c = atof(val.c_str());
                 x = 0;
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
    bool enableCOM = false; //Now in main, true here will take over main thread
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

    double Control::GetMotorPreviousPositionDegrees()
    {
        return motor->GetPreviousPositionDegrees();
    }

    void Control::SetMotorPreviousPositionDegrees(double val)
    {
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

    std::vector<Point3D> Control::GetLaserScan()
    {
        return Control::mpLasercallback.mLaserScan;
    }

    ///
    /// Scan Management Functions
    ///
    pcl_data Control::GetIncompleteScan()
    {
        return PublicPartialScan;
    }

    pcl_data Control::GetCompleteScan()
    {
        return PublicScan;
    }

    void Control::SetCompleteScan(pcl_data data)
    {
        PublicScan = data;
    }

    void Control::SetIncompleteConstruction(pcl_data data)
    {
        PublicPartialScan = data;
    }

    void Control::ClearIncompleteScan()
    {
        PublicPartialScan.id = 0;
        PublicPartialScan.message = "";
        PublicPartialScan.points.clear();
        PublicPartialScan.scancount = 0;
    }

    void Control::AddToScanConstruction(std::vector<Point3D> laserscan,
                                        double currentMotorPosition,
                                        double previousMotorPosition)
    {
        int scancnt = laserscan.size();//1080;
        //Check for complete scan & get delta
        double delta_position = 0.0;
        bool scancomplete = false;
        if(currentMotorPosition<previousMotorPosition)
        {
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
            point.r = (laserscan[i]).GetX();
            point.theta = laserscan[i].GetY();
            point.phi = previousMotorPosition + (i*(delta_position/scancnt));
            PublicPartialScan.points.push_back(point);
        }

        PublicPartialScan.scancount++;

        // If complete set the complete scan and copy to globally
        // accessible object
        if(scancomplete)
        {
            SetCompleteScan(PublicPartialScan);
            ClearIncompleteScan();
        }
}


/*End of File */
