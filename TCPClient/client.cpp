////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////

#define CLIENT_CPP
#ifdef CLIENT_CPP

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../Pidar/connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "../Pidar/pointstructs.hpp"

using namespace PointCloud;


namespace ClientSide {


std::vector<pcl_data> Latest_Clouds_;
std::string Port = "10000";
std::string Address = "localhost";

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

        boost::asio::ip::tcp::resolver resolver(io_service);
            boost::asio::ip::tcp::resolver::query query(host, service);
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
              resolver.resolve(query);
            boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;


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
            Latest_Clouds_ = clouds_;
        }
        else
        {
            // An error occurred.
            std::cerr << "ERROR: " << e.message() << std::endl;
        }

        //TODO: Create callback function here that updates data
        // This is the endpoint for the connection
    }

    /// Handle completion of a read operation.
    void handle_read(const boost::system::error_code& e)
    {
        connection_.async_read(clouds_,
                               boost::bind(&client::handle_data, this,
                                           boost::asio::placeholders::error));
    }

    /// Handle completion of a connect operation.
      void handle_connect(const boost::system::error_code& e,
          boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
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
        else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator())
        {
          // Try the next endpoint.
          connection_.socket().close();
          boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
          connection_.socket().async_connect(endpoint,
              boost::bind(&client::handle_connect, this,
                boost::asio::placeholders::error, ++endpoint_iterator));
        }
        else
        {
          // An error occurred. Log it and return. Since we are not starting a new
          // operation the io_service will run out of work to do and the client will
          // exit.
          std::cerr << e.message() << std::endl;
        }
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

void setServer(std::string add, std::string prt){
    Address = add;
    Port = prt;

    //Put in empty cloud if empty cloud found.
    if(Latest_Clouds_.size()<1)
    {
        pcl_data blank;
        blank.id = -1;
        Latest_Clouds_.push_back(blank);
    }
}

pcl_data sendCommand(int cmd){
    boost::asio::io_service io_service;
    ClientSide::client client(io_service, Address, Port,cmd,0);
    io_service.run();


    std::cout<<"Received ID: "<<Latest_Clouds_[0].id<<std::endl;
    std::cout<<"Received Message: "<<Latest_Clouds_[0].message<<std::endl;

    return Latest_Clouds_[0];
}


pcl_data getLatestCloud(){

    return sendCommand(1);
}

pcl_data getLatestSpeed(){

    return sendCommand(2);
}


}

#endif
