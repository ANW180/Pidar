/**
  \file Main.cpp
  \brief Main file for running the Pidar program.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
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
#include <math.h>


#define MULTICASTGROUP "239.255.0.1"

// Allocating and initializing the globals.
Pidar::Control* Pidar::Control::mpInstance = 0;
std::deque<Pidar::pcl_data> gSendPoints;
unsigned short gLEDCount = 0;
bool gStopFlag = false;
bool gISRFlag = false;

using namespace std;
using namespace Pidar;


int main()
{
    if(wiringPiSetupSys() < 0)
    {
#ifdef DEBUG
        std::cout<< "Unable to setup wiringPi: "<< strerror(errno) << std::endl;
#endif
        return -1;
    }
    while(!Control::Instance()->Initialize())
    {
        sleep(1);
    }
    unsigned int restartCnt = 0;
    sleep(1);

    // Start the UDP server
    boost::asio::io_service ioService;
    Server s(ioService,
             boost::asio::ip::address::from_string(MULTICASTGROUP));
    boost::thread serverThread(boost::bind(&boost::asio::io_service::run,
                                 &ioService));

    // Start the UDP command server
    boost::asio::io_service ioServiceCommands;
    CommandServer cmdsrv(ioServiceCommands, 10001);
    boost::thread commandThread(boost::bind(&boost::asio::io_service::run,
                                  &ioServiceCommands));
    while(1)
    {
        if(gISRFlag)
        {
            if(restartCnt > 0)
            {
#ifdef DEBUG
                std::cout << "ISR restarted "
                          << restartCnt
                          << " times"
                          << std::endl;
#endif
            }
            gISRFlag = false;
        }
        else
        {
#ifdef DEBUG
            std::cout << "ISR NOT running, restarting "
                      << restartCnt
                      << " restart attempts made."
                      << std::endl;
#endif
            Control::Instance()->SetupWiringPi();
            restartCnt++;
        }
        // Fix for laser disconnecting on slipring?
        // Shutdown thread and serial connection, sleep, then
        // restart to resume data output.
        if(Control::Instance()->mpLaser->GetErrorCount() > 2)
        {
#ifdef DEBUG
            std::cout << "Restarting Laser Connection" << std::endl;
#endif
            Control::Instance()->mpLaser->Shutdown();
            sleep(1);
            Control::Instance()->mpLaser->Initialize();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
    return 0;
}
/** End of File */
