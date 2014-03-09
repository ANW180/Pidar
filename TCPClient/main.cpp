////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////

#define CLIENT
#ifdef CLIENT

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../Pidar/connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "../Pidar/pointcloud.hpp"
#include <client.cpp>
#include <visual.cpp>


int main()
{
    try
    {

        ClientSide::setServer("localhost","10000"); //not needed unless changing values

        //pcl_data mycloud = ClientSide::getLatestCloud();
        //std::cout << "id: " << mycloud.id     << std::endl;

        //mycloud = ClientSide::getLatestSpeed();
        //std::cout << "id: " << mycloud.id     << std::endl;


        //TODO: Create class to handle received data
        //TODO: Create function to send different commands

        Render::Visual* newvisual = new Render::Visual();
        newvisual->show();

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}


#endif
