////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for testing different system classes
///        and functionality.
/// Author: Andrew Watson, Jonathan Ulrich
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com, JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include "hokuyo.hpp"
#include "dynamixel.hpp"
#include "control.hpp"
#include "global.hpp"
#include <wiringPi.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;
Pidar::Control* maincontrol;

int main()
{
    maincontrol = new Pidar::Control();
    while(!maincontrol->Initialize())
    {
        sleep(1);
    }
    globMotorSpeed = 1;
    int restartCnt = 0;
    maincontrol->StartMotor(globMotorSpeed);
    sleep(1);

    //Start Web Server Thread
    boost::asio::io_service io_service;
    PointCloud::server s(io_service,
                         boost::asio::ip::address::from_string("239.255.0.1"));
    boost::thread bt(boost::bind(&boost::asio::io_service::run,
                                 &io_service));

    //Start Seperate Command Web Server
    boost::asio::io_service io_service_cmds;
    commandserver cmdsrv(io_service_cmds, 10001);
    boost::thread bt2(boost::bind(&boost::asio::io_service::run,
                                  &io_service_cmds));
    while(1)
    {
        if(ISRrunning)
        {
            if(restartCnt > 0)
            {
                std::cout << "ISR restarted "
                          << restartCnt
                          << " times"
                          << std::endl;
            }
            ISRrunning = false;
        }
        else
        {
            std::cout << "ISR NOT running, restarting "
                      << restartCnt
                      << " restart attempts made."
                      << std::endl;
            maincontrol->StartISR();
            boost::this_thread::sleep(
                        boost::posix_time::millisec(1000));
            restartCnt++;
        }
        //Check for updates from client
        if(globFoundUpdate)
        {
            maincontrol->StartMotor(globMotorSpeed);
            globFoundUpdate = false;

            //If stopping motor, stop whole program
            if(globMotorSpeed == 0)
                return 0;
        }
        // Fix for laser disconnecting on slipring?
        // Shutdown thread and serial connection, sleep, then
        // restart both to resume data output.
        if(maincontrol->GetLaserPtr()->GetErrorCount() > 2)
        {
            std::cout << "Restarting Laser Connection" << std::endl;
            maincontrol->GetLaserPtr()->Shutdown();
            sleep(1);
            maincontrol->GetLaserPtr()->Initialize();
            sleep(1);
        }
        sleep(1);
    }
   return 0;
}
/** End of File */
