/**
  \file Main.cpp
  \brief Main file for running the Pidar program.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#include "Hokuyo.hpp"
#include "Dynamixel.hpp"
#include "Control.hpp"
#include "Global.hpp"
#include <wiringPi.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;
using namespace Pidar;


Pidar::Control* gMainControl;


int main()
{
    gMainControl = new Pidar::Control();
    while(!gMainControl->Initialize())
    {
        sleep(1);
    }
    gMotorSpeed = 1.;
    int restartCnt = 0;
    sleep(1);

    //Start Web Server Thread
    boost::asio::io_service io_service;
    Server s(io_service,
             boost::asio::ip::address::from_string("239.255.0.1"));
    boost::thread bt(boost::bind(&boost::asio::io_service::run,
                                 &io_service));

    //Start Seperate Command Web Server
    boost::asio::io_service io_service_cmds;
    CommandReceiver cmdsrv(io_service_cmds, 10001);
    boost::thread bt2(boost::bind(&boost::asio::io_service::run,
                                  &io_service_cmds));
    while(1)
    {
        if(gISRFlag)
        {
            if(restartCnt > 0)
            {
                //                std::cout << "ISR restarted "
                //                          << restartCnt
                //                          << " times"
                //                          << std::endl;
            }
            gISRFlag = false;
        }
        else
        {
            //            std::cout << "ISR NOT running, restarting "
            //                      << restartCnt
            //                      << " restart attempts made."
            //                      << std::endl;
            gMainControl->StartISR();
            restartCnt++;
        }
        //Check for updates from client
        if(gFoundUpdate)
        {
            gMainControl->mMotor->SetSpeedRpm(gMotorSpeed, true);
            gFoundUpdate = false;
        }
        // Fix for laser disconnecting on slipring?
        // Shutdown thread and serial connection, sleep, then
        // restart to resume data output.
        if(gMainControl->mLaser->GetErrorCount() > 2)
        {
            //            std::cout << "Restarting Laser Connection" << std::endl;
            gMainControl->mLaser->Shutdown();
            sleep(1);
            gMainControl->mLaser->Initialize();
        }
        usleep(100000);
    }
    return 0;
}
/** End of File */
