#ifndef CONNECTIONCLASS
#define CONNECTIONCLASS
#include "pointstructs.hpp"
#include "point3d.hpp"
#include "control.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>
#include <iostream>

namespace PointCloud
{
const short multicast_port = 30001;
const int max_message_count = 1000;

    class server
    {
    public:
      server(boost::asio::io_service& io_service,
          const boost::asio::ip::address& multicast_address)
        : endpoint_(multicast_address, multicast_port),
          socket_(io_service, endpoint_.protocol()),
          timer_(io_service)
      {
        clouds_.clear();
            pcl_data w;
            pcl_point x;
            x.r = 0.0;
            x.theta = 0.0;
            x.phi = 0.0;
            w.points.push_back(x);
            clouds_.push_back(w);
        std::cout<<"Sending Data"<<std::endl;
        std::cout<<"on "<<endpoint_.address()<<":"<<endpoint_.port()<<std::endl;
        socket_.async_send_to(
            boost::asio::buffer(clouds_), endpoint_,
            boost::bind(&server::handle_send_to, this,
              boost::asio::placeholders::error));
      }

      void handle_send_to(const boost::system::error_code& error)
      {
        if (!error)
        {
          //wait one millisecond
          timer_.expires_from_now(boost::posix_time::milliseconds(1));
          timer_.async_wait(
              boost::bind(&server::handle_timeout, this,
                boost::asio::placeholders::error));
        }
      }

      void handle_timeout(const boost::system::error_code& error)
      {
        if (!error)
        {
          clouds_.clear();
          if(!SendPoints.empty()){
              clouds_.push_back(SendPoints.front());
              SendPoints.pop_front();
//              std::cout<<"Sending Public Scan"<<std::endl;
//              std::cout<<"on "<<endpoint_.address()<<":"<<endpoint_.port()<<std::endl;
              socket_.async_send_to(
                          boost::asio::buffer(clouds_[0].points), endpoint_,
                  boost::bind(&server::handle_send_to, this,
                    boost::asio::placeholders::error));
          }
        }
      }

    private:
      boost::asio::ip::udp::endpoint endpoint_;
      boost::asio::ip::udp::socket socket_;
      boost::asio::deadline_timer timer_;
      std::vector<pcl_data> clouds_;
    };

}

#endif
