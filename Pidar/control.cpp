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
std::deque<pcl_data> SendPoints;
int globMotorSpeed = 0;
bool globFoundUpdate = false;

using namespace Pidar;
using namespace PointCloud;
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
timespec t1, t2;
int i = 0;

void InterruptService(void)
{

    //clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
    //Get Current and Previous Positions
    current_position = maincontrol->GetMotorPositionDegrees();
    previous_position = maincontrol->GetMotorPreviousPositionDegrees();
    if(fabs(current_position - previous_position) < 0.0001)
    {
        std::cout << "Bad Servo Read ISR #: " << i << std::endl;
        i++;
        return;
    }
    //Update Previous Motor Position
    maincontrol->SetMotorPreviousPositionDegrees(current_position);

    //Get Values to send for construction
    laserscan = maincontrol->GetLaserScan();

    //Send values and get newly constructed incompletescan
    //this will update values as needed
    maincontrol->AddToScanQueue(laserscan,
                                      current_position,previous_position);
    //clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
    //std::cout << diff(t1, t2).tv_sec << " " << diff(t1, t2).tv_nsec << std::endl;
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

//    ///******* TESTING ONLY **********************************
    //This is for testing only (remove when not testing)
//    if(!enableISR)
//    {
//        pcl_data tmpscan = GetSampleTXTFileData();
//        tmpscan.id = 1; //initialization
//        tmpscan.speed = 88;
//        for(int i = 0; i<tmpscan.points.size()-1080;i+=1080)
//        {
//            pcl_data x;
//            for(int j = 0;j<1080;j++)
//            {
//             pcl_point y = tmpscan.points[i+j];
//             x.points.push_back(y);
//            }
//            SendPoints.push_back(x);
//        }
//        std::cout<<"Queue Length: "<<SendPoints.size()<<std::endl;
//        std::cout<<"control.cpp: ID: "<<tmpscan.id<<std::endl;
//        std::cout<<"control.cpp: r: "<<tmpscan.points[0].r<<std::endl;
//        std::cout<<"control.cpp: theta: "<<tmpscan.points[0].theta<<std::endl;
//        std::cout<<"control.cpp: phi: "<<tmpscan.points[0].phi<<std::endl;
//    }

//    ///******** END TESTING LINES ****************************


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
        if (wiringPiSetupSys () < 0)
        {
            fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
            //   return 1;
        }
        // set Pin 17/0 generate an interrupt on low-to-high transitions
        // and attach myInterrupt() to the interrupt
        if ( wiringPiISR (HOKUYOSYNCPIN, INT_EDGE_FALLING, InterruptService) < 0 )
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
            //   return 1;
        }

    }

    if(enableCOM)
    {
        try
        {
            //Start Web Server Thread
            boost::asio::io_service io_service;
            PointCloud::server s(io_service, boost::asio::ip::address::from_string("239.255.0.1"));
            boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service));

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
        sleep(1000);
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


    void Control::AddToScanQueue(std::vector<Point3D> laserscan,
                                        double currentMotorPosition,
                                        double previousMotorPosition)
    {
        int scancnt = laserscan.size();//1080;
        //Check for complete scan & get delta
        double delta_position = 0.0;
        if (previousMotorPosition < currentMotorPosition)
        {
            delta_position = ((360-previousMotorPosition)-currentMotorPosition);
        }
        else
        {
            delta_position = fabs(currentMotorPosition-previousMotorPosition);
        }

        pcl_data tmp;
        if(laserscan.size() > 0)
        {
            for(int i = 0; i <= scancnt; i++)
            {
                pcl_point point;
                point.r = (laserscan[i]).GetX();
                point.theta = (laserscan[i]).GetY();
                point.phi = previousMotorPosition + (i*(delta_position/scancnt));
                tmp.points.push_back(point);
            }
            //Add scan to queue
            SendPoints.push_back(tmp);
        }
}


/*End of File */
