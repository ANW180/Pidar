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

Control::Control(){
    motor = new Motor::Dynamixel();
    laser = new Laser::Hokuyo();
}

void LaserISR(void){
    //TODO: stuff
}

bool Control::Initialize(){
    laser->RegisterCallback(mpLasercallback);
    motor->RegisterCallback(mpMotorcallback);

    laser->Initialize();
    motor->Initialize();

//    if (wiringPiSetup () < 0) {
//          fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
//          return 1;
//      }

//      // set Pin 17/0 generate an interrupt on low-to-high transitions
//      // and attach myInterrupt() to the interrupt
//      if ( wiringPiISR (HOKUYOSYNCPIN, INT_EDGE_RISING, &LaserISR) < 0 ) {
//          fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
//          return 1;
//      }


//      try
//      {
//        unsigned short port = boost::lexical_cast<unsigned short>("10000");

//        boost::asio::io_service io_service;

//        pointcloud_connection::server server(io_service, port);
//        io_service.run();
//      }
//      catch (std::exception& e)
//      {
//        std::cerr << e.what() << std::endl;
//      }

    return true;

}

double Motor::Dynamixel::GetPositionDegrees(){}









