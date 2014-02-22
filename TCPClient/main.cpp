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
      const std::string& host, const std::string& service)
    : connection_(io_service)
  {
    // Resolve the host name into an IP address.
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(host, service);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
      resolver.resolve(query);

    // Start an asynchronous connect operation.
    boost::asio::async_connect(connection_.socket(), endpoint_iterator,
        boost::bind(&client::handle_connect, this,
          boost::asio::placeholders::error));
  }

  /// Handle completion of a connect operation.
  void handle_connect(const boost::system::error_code& e)
  {
    if (!e)
    {
      // Successfully established connection. Start operation to read the list
      // of stocks. The connection::async_read() function will automatically
      // decode the data that is read from the underlying socket.
      connection_.async_read(clouds_,
          boost::bind(&client::handle_read, this,
            boost::asio::placeholders::error));
    }
    else
    {
      // An error occurred. Log it and return. Since we are not starting a new
      // operation the io_service will run out of work to do and the client will
      // exit.
      std::cerr << e.message() << std::endl;
    }
  }

  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e)
  {
    if (!e)
    {
      // Print out the data that was received.
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

    // Since we are not starting a new operation the io_service will run out of
    // work to do and the client will exit.
  }

private:
  /// The connection to the server.
  connection connection_;

  /// The data received from the server.
  std::vector<pcl_data> clouds_;
};

} // namespace s11n_example

int main(int argc, char* argv[])
{
  try
  {
    // Check command line arguments.
    if (argc != 3)
    {
      std::cerr << "Usage: client <host> <port>" << std::endl;
      //return 1;
     //   std::cerr << "Default Client to localhost:10000" << std::endl;
        argv[1]="localhost";
        argv[2]="10000";
    }

    boost::asio::io_service io_service;
    pointcloud_connection::client client(io_service, "localhost", "10000");
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

#endif


//#define TESTCLIENT
#ifdef TESTCLIENT

#include <time.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <fstream>
/**
 *http://julien.boucaron.free.fr/wordpress/?p=178
 */


using boost::asio::ip::tcp;

int main()
{
  try
  {

    const char *port = "10000"; //the port we connect
    const unsigned int buff_size = 65536; //the size of the read buffer


    boost::asio::io_service io_service; //asio main object
    tcp::resolver resolver(io_service); //a resolver for name to @
    tcp::resolver::query query("localhost", port); //ask the dns for this resolver
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query); //iterator if multiple answers for a given name
    tcp::resolver::iterator end;

    tcp::socket socket(io_service); //attach a socket to the main asio object
    socket.connect(*endpoint_iterator); //connect to the first returned object


    unsigned int count = 0; //a counter
    while(1) { //loop until (see break after)
      boost::array<char, buff_size> buf; //create read buffer
      boost::system::error_code error; //in case of error
      size_t len = socket.read_some(boost::asio::buffer(buf), error); //read data
      std::cout << "Read " << len <<  std::endl;
      count += len;
      std::cout << "Read a total of " << count << " bytes " << std::endl;
      if (error == boost::asio::error::eof ) { //if end of file reached


      }
      else if (error) {
        throw boost::system::system_error(error); // Some other error.
      }
      else {

            //Create matching data structure
            //len is the length of received information
            double * data = new double[len];

            //Copy received data into data structure
            //memcpy(data, buf.data(), sizeof(double) * len);
            memcpy(data, buf.data(), len);

            //Output data to screen
            std::cout<<"Begin data"<<std::endl;
            for(int i = 0;i<(8000);i++)
                std::cout<<data[i]<<std::endl;
            std::cout<<"End data"<<std::endl<<"Length: "<<len<<std::endl;

//            if (error == boost::asio::error::eof ) { //if end of file reached
//                f.write(buf.data(),len); //finish write data
//                f.close();   break; // Connection closed cleanly by peer.
//                  }
            break; //quit while(1) loop
      }
    }
  }
  catch (std::exception& e)
  {
    std::cout << "Exception: " << e.what() << std::endl;
  }

  return 0;
}

#endif

