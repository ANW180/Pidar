#ifndef CONNECTIONCLASS
#define CONNECTIONCLASS

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <control.hpp>
#include <connection.hpp> // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include <vector>
#include <pointcloud.hpp>
#include <pointstructs.hpp>

namespace PointCloud {

class server
{
public:

    //This will choose what happends with each command from the client
    pcl_data ProcessCommands(std::vector<pcl_commands> cmd){

        pcl_data s;
        int id = (int)cmd[0].cmd;
        switch ( id ) {
        case 0:
          break;
        case 1:
            s = PublicScan;
            std::cout<<"connection.cpp: ID: "<<s.id<<std::endl;
            s.message = "Full Scan Returned";
          break;

        case 2:
            s.id = 2;
            std::cout<<"Command 2 requested: Speed"<<std::endl;
            s.id = 1;
            s.speed = PublicScan.speed;
            //PublicScan.speed ++;
            s.message = "Speed only returned";
          break;

        default:
            s.id = 99;
            std::cout<<"Other command requested"<<std::endl;
            s.message = "No command of this type exists... Sorry.";
          break;
        }

        return s;

    }

  /// Constructor opens the acceptor and starts waiting for the first incoming
  /// connection.
  server(boost::asio::io_service& io_service, unsigned short port)
    : acceptor_(io_service,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
  {
    // Create the data to be sent to each client.

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a accept operation.
  void handle_accept(const boost::system::error_code& e, connection_ptr conn)
  {
    if (!e)
    {
            conn->async_read(commands_,
                  boost::bind(&server::handle_read,this,
                  boost::asio::placeholders::error,conn));
    }

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(acceptor_.get_io_service()));
    acceptor_.async_accept(new_conn->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));
  }

  /// Handle completion of a write operation.
  void handle_write(const boost::system::error_code& e, connection_ptr conn)
  {
      //After sending the cloud, clear the vector
      clouds_.clear();

    // Nothing to do. The socket will be closed automatically when the last
    // reference to the connection object goes away.
  }

  void fillarray(double *arr, int size){
      for(int i = 0;i<size;i++)
      {
          arr[i]=i;
      }

  }


  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e, connection_ptr conn)
  {
      //TODO: make class to react to commands

    if (!e)
    {
        //Get return data
        clouds_.clear();
        pcl_data ret = ProcessCommands(commands_);
        clouds_.push_back(ret);

        //Send back the data
        conn->async_write(clouds_,
            boost::bind(&server::handle_write, this,
              boost::asio::placeholders::error, conn));
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
  /// The acceptor object used to accept incoming socket connections.
  boost::asio::ip::tcp::acceptor acceptor_;

  /// The data to be sent to each client.
  std::vector<pcl_data> clouds_;
  std::vector<pcl_commands> commands_;
};


}

#endif
