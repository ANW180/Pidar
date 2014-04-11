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
bool ISRrunning = true;
unsigned int ledCount = 0;


using namespace Pidar;
using namespace PointCloud;


void InterruptServiceStop(void)
{
    maincontrol->StopMotor();
}


void InterruptService(void)
{
    if(ledCount == 0)
    {
        digitalWrite(LEDPIN, 1);
    }
    if(ledCount == 1)
    {
        digitalWrite(LEDPIN, 0);
    }
    ledCount++;
    if(ledCount == 2)
    {
        ledCount = 0;
    }
    ISRrunning = true;
    //Get Current and Previous Positions
    //isMotorConnected also acts as a keepalive signal
    if(maincontrol->isMotorConnected())
    {
        current_position = maincontrol->GetMotorPositionDegrees();
        previous_position = maincontrol->GetMotorPreviousPositionDegrees();
        if(current_position == previous_position)
        {
//            std::cout.precision(15);
//            std::cout << "Angle Diff: "
//                      << fabs(current_position - previous_position)
//                      << std::endl;
            return;
        }
        //Update Previous Motor Position
        maincontrol->SetMotorPreviousPositionDegrees(current_position);

        //Get Values to send for construction
        laserscan = maincontrol->GetLaserScan();

        //Send values and get newly constructed incompletescan
        //this will update values as needed
        maincontrol->AddToScanQueue(laserscan,
                                    current_position,
                                    previous_position);
    }
    else
    {
        std::cout << "Motor is disconnected" << std::endl;
    }
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
    bool enableISR = true;

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
    sleep(1);
    if(enableISR)
    {
        if (wiringPiSetupSys () < 0)
        {
            fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
            //   return 1;
        }
        // set Pin 17/0 generate an interrupt on low-to-high transitions
        // and attach myInterrupt() to the interrupt
        if (wiringPiISR(HOKUYOSYNCPIN, INT_EDGE_RISING, InterruptService) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        if (wiringPiISR(SHUTDOWNPIN, INT_EDGE_RISING, InterruptServiceStop) < 0)
        {
            fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        }
        // Setup LED pin mode
        system("gpio export 10 out");
        sleep(1);
        pinMode(LEDPIN, OUTPUT);
        sleep(1);
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

bool Control::isMotorConnected()
{
    return motor->IsConnected();
}

void Control::StartMotor(int rpm)
{
     Control::motor->SetSpeedRpm(rpm, true);
}

void Control::StopMotor()
{
    Control::motor->SetSpeedRpm(0, true);
    sleep(1);
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
    if(previousMotorPosition < currentMotorPosition)
    {
        delta_position = ((360.0 - previousMotorPosition) -
                         currentMotorPosition);
    }
    else
    {
        delta_position = fabs(currentMotorPosition - previousMotorPosition);
    }

    pcl_data tmp;
    if(laserscan.size() > 0)
    {
        for(int i = 0; i <= scancnt; i++)
        {
            pcl_point point;
            point.r = (laserscan[i]).GetX();
            point.theta = (laserscan[i]).GetY();
            point.phi = 360.0 - double(previousMotorPosition +
                                (i * (delta_position / scancnt)));
            tmp.points.push_back(point);
        }
        //Add scan to queue
        SendPoints.push_back(tmp);
        }
}

bool Control::StartISR()
{
    bool result = true;
    if (wiringPiSetupSys () < 0)
    {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        result = false;
        //   return 1;
    }
    // set Pin 17/0 generate an interrupt on low-to-high transitions
    // and attach myInterrupt() to the interrupt
    if (wiringPiISR (HOKUYOSYNCPIN, INT_EDGE_RISING, InterruptService) < 0)
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        result = false;
        //   return 1;
    }
    return result;
}


/*End of File */
