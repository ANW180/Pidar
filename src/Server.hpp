/**
  \file Server.hpp
  \brief UDP server interface class for connecting to the Pidar.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "PointStructs.hpp"
#include "Point3D.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>
#include <iostream>


namespace Pidar
{
    /**
     * @brief The Server class implements the boost UDP interface to enable
     * broadcasting of collected 3D scan data.
     */
    class Server
    {
    public:
        static const short mMulticastPort;
        static const int mTransmissionDelayMs;
        Server(boost::asio::io_service& ioService,
               const boost::asio::ip::address& multicastAddress);
        void HandleSend(const boost::system::error_code& error);
        void HandleTimeout(const boost::system::error_code& error);

    private:
        boost::asio::ip::udp::endpoint mEndpoint;
        boost::asio::ip::udp::socket mSocket;
        boost::asio::deadline_timer mDeadlineTimer;
        std::vector<pcl_data> mPointClouds;
    };
}
/* End of File */
