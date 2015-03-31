/**
  \file Server.hpp
  \brief UDP server interface class for connecting to the Pidar.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#pragma once
#include "PointStructs.hpp"
#include "Point3D.hpp"
#include "Control.hpp"
#include "Global.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>
#include <iostream>


namespace Pidar
{
    const short multicast_port = 30001;
    const int max_message_count = 1000;
    const int transmission_delay_ms = 1; //milliseconds

    class Server
    {
    public:
        Server(boost::asio::io_service& io_service,
               const boost::asio::ip::address& multicast_address);
        void handle_send_to(const boost::system::error_code& error);
        void handle_timeout(const boost::system::error_code& error);

    private:
        boost::asio::ip::udp::endpoint mEndpoint;
        boost::asio::ip::udp::socket mSocket;
        boost::asio::deadline_timer mDeadlineTimer;
        std::vector<pcl_data> mPointClouds;
    };
}
/* End of File */
