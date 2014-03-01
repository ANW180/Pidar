#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "pointcloud.hpp"

namespace pointcloud_connection {

void fillarray(double *arr, int size){
    for(int i = 0;i<size;i++)
    {
        arr[i]=i;
    }

}

class server
{
public:
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

  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e, connection_ptr conn)
  {
      //TODO: make class to react to commands
 int mycommand = 0; //init to 0
    if (!e)
    {
        //only read first vector received [0], multiple vectors may allow for multiple commands and data
        std::cout << "    cmd: " << commands_[0].cmd<< "\n";
        mycommand = (int) commands_[0].cmd;

        //TODO: create seperate class for protocol logic
        if(mycommand == 1){
        pcl_data s;
        s.id = 1;
        fillarray(s.x,1080);
        fillarray(s.y,1080);
        fillarray(s.z,1080);
        clouds_.push_back(s);
        }
        else if(mycommand == 2)
        {
        pcl_data s;
        s.id = 2;
        fillarray(s.x,1080);
        fillarray(s.y,1080);
        fillarray(s.z,1080);
        clouds_.push_back(s);
        }
        else{
            pcl_data s;
            s.id = -1;
            fillarray(s.x,1080);
            fillarray(s.y,1080);
            fillarray(s.z,1080);
            clouds_.push_back(s);
        }

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
