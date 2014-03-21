////////////////////////////////////////////////////////////////////////////////
///
/// \file client.cpp
/// \brief Client class source file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../Pidar/connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "../Pidar/pointstructs.hpp"

using namespace PointCloud;

std::vector<pcl_data> Latest_Clouds_;
std::deque<pcl_data> SendPoints;
bool lock_write;
bool lock_clear;
bool lock_read;
const short multicast_port = 30001;

class receiver
{
public:
  receiver(boost::asio::io_service& io_service,
      const boost::asio::ip::address& listen_address,
      const boost::asio::ip::address& multicast_address)
    : socket_(io_service)
  {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, multicast_port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&receiver::handle_receive_from, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));



  }

  void handle_receive_from(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    if (!error)
    {
      //std::cout.write(data_, bytes_recvd);
     // std::cout << std::endl;
      //Generate PCL_POINT vector from buffer

      int points_rcvd;
      points_rcvd = (bytes_recvd / 8) / 3;
      pcl_data dat;
      for(int k = 0;k<bytes_recvd;k+=24)
      {
          pcl_point tmp;
          double d;
          char subdata_r[8];
          char subdata_theta[8];
          char subdata_phi[8];

          for(int i = 0;i<8;i++)
          {
              subdata_r[i] = data_[i + k];
              subdata_theta[i] = data_[i + k + 8];
              subdata_phi[i] = data_[i + k + 8 + 8];
          }

          tmp.r = *reinterpret_cast<double*>(subdata_r) ;
          tmp.theta = *reinterpret_cast<double*>(subdata_theta) ;
          tmp.phi = *reinterpret_cast<double*>(subdata_phi) ;
          dat.points.push_back(tmp);
      }

      while(lock_clear || lock_read);
      lock_write = true;
      SendPoints.push_back(dat);
      lock_write = false;

//    std::cout<<"Clouds_ size: "<<dat.points.size()<<std::endl;
//    std::cout<<"X: "<<dat.points[0].r<<std::endl;
//    std::cout<<"Theta: "<<dat.points[0].theta<<std::endl;
//    std::cout<<"Phi: "<<dat.points[0].phi<<std::endl;
  //  clouds_.clear();
   //   std::cout << std::endl;

      socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          boost::bind(&receiver::handle_receive_from, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
  }

private:
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  enum { max_length = 50000 };
  char data_[max_length];
  std::vector<pcl_point> clouds_;
};
