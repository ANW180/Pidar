////////////////////////////////////////////////////////////////////////////////
///
/// \file server.cpp
/// \brief UDP server interface class for connecting to the Pidar.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
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
    const int transmission_delay_ms = 1; //milliseconds

    class Server
    {
    public:
        Server(boost::asio::io_service& io_service,
               const boost::asio::ip::address& multicast_address)
            : mEndpoint(multicast_address, multicast_port),
              mSocket(io_service, mEndpoint.protocol()),
              mDeadlineTimer(io_service)
        {
            mPointClouds.clear();
            pcl_data w;
            pcl_point x;
            x.r = 0.0;
            x.theta = 0.0;
            x.phi = 0.0;
            x.intensity = 0.0;
            x.newScan = false;
            w.points.push_back(x);
            mPointClouds.push_back(w);
        std::cout << "Sending Data"
                  << std::endl;
        std::cout << "on "
                  << mEndpoint.address()
                  << ":" << mEndpoint.port()
                  << std::endl;
        mSocket.async_send_to(
            boost::asio::buffer(mPointClouds), mEndpoint,
            boost::bind(&Server::handle_send_to, this,
              boost::asio::placeholders::error));
        }


        void handle_send_to(const boost::system::error_code& error)
        {
            if(!error)
            {
                mDeadlineTimer.expires_from_now
                        (boost::posix_time::milliseconds(transmission_delay_ms));
                mDeadlineTimer.async_wait(
                    boost::bind(&Server::handle_timeout, this,
                        boost::asio::placeholders::error));
            }
        }


        void handle_timeout(const boost::system::error_code& error)
        {
            if(!error)
            {
                mPointClouds.clear();
                if(gSendPoints.size() > 5)
                {
                    gSendPoints.clear();
                }
                if(!gSendPoints.empty())
                {
                    mPointClouds.push_back(gSendPoints.front());
                    gSendPoints.pop_front();
                    mSocket.async_send_to(
                          boost::asio::buffer(mPointClouds[0].points),
                                        mEndpoint,
                            boost::bind(&Server::handle_send_to,
                                        this,
                                        boost::asio::placeholders::error));
                }
                else
                {
                    handle_send_to(error);
                }
            }
        }

    private:
        boost::asio::ip::udp::endpoint mEndpoint;
        boost::asio::ip::udp::socket mSocket;
        boost::asio::deadline_timer mDeadlineTimer;
        std::vector<pcl_data> mPointClouds;
    };
}
/* End of File */
