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
//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../Pidar/connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "../Pidar/pointcloud.hpp"

namespace pointcloud_connection {

/// Downloads stock quote information from a server.
class client
{
public:

  /// Constructor starts the asynchronous connect operation.
  client(boost::asio::io_service& io_service,
      const std::string& host, const std::string& service, int newcmd, double newval)
    : connection_(io_service)
  {
      send_command = newcmd;
      send_value = newval;
    // Resolve the host name into an IP address.
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(host, service);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
      resolver.resolve(query);

    // Start an asynchronous connect operation.
    boost::asio::async_connect(connection_.socket(), endpoint_iterator,
        boost::bind(&client::handle_write, this,
          boost::asio::placeholders::error));
  }


  void handle_write(const boost::system::error_code& e){

      //TODO: Command request logic here { }

      pcl_commands command;
      command.cmd = send_command;
      std::cout << "sending cmd: " << command.cmd << std::endl;
      commands_.push_back(command);

      if(!e){

      connection_.async_write(commands_,
          boost::bind(&client::handle_read, this,
            boost::asio::placeholders::error));
      }
      else
      {
          std::cout << "crap?: " << e << std::endl;
      }
  }

  void handle_data(const boost::system::error_code& e){

      if (!e)
      {
        // Print out the data that was received.
         //TODO: Create class to handle received data
        for (std::size_t i = 0; i < clouds_.size(); ++i)
        {
          std::cout << "Cloud number " << i << "\n";
          std::cout << "    code: " << clouds_[i].id << "\n";
          std::cout << "       x: " << clouds_[i].x[1079] << "\n";
          std::cout << "       y: " << clouds_[i].y[3] << "\n";
          std::cout << "       z: " << clouds_[i].z[4] << "\n";
        }
      }
      else
      {
        // An error occurred.
        std::cerr << "ERROR: " << e.message() << std::endl;
      }
  }

  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e)
  {
      connection_.async_read(clouds_,
          boost::bind(&client::handle_data, this,
            boost::asio::placeholders::error));
  }

private:
  /// The connection to the server.
  connection connection_;
  int send_command;
  double send_value;

  /// The data received from the server.
  std::vector<pcl_data> clouds_;
  std::vector<pcl_commands> commands_;
};

}

int main(int argc, char* argv[])
{
  try
  {
    // Check command line arguments.
    if (argc != 3)
    {
      std::cerr << "Usage: client <host> <port>" << std::endl;
      //return 1;
        std::cerr << "Default Client to localhost:10000" << std::endl;
        argv[1]="localhost";
        argv[2]="10000";
    }

    //Only connect when you want something, run will close when finished
    for(int i = 0; i< 3;i++){
        boost::asio::io_service io_service;
        pointcloud_connection::client client(io_service, "localhost", "10000",i,0);
        io_service.run();
    }


    //TODO: Create function to send different commands

  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

#endif
