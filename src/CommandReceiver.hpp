/**
  \file CommandReceiver.hpp
  \brief UDP interface for controlling the Pidar parameters.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#pragma once
#include "PointStructs.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define MAX_RPM 27
#define MAX_LENGTH 1024


namespace Pidar
{
    class CommandReceiver
    {
    public:
        CommandReceiver(boost::asio::io_service& ioService, short port);
        std::string HandleCommands(char data[], size_t bytesReceived);
        void HandleReceieve(const boost::system::error_code& error, size_t bytesReceived);
        void HandleSend(const boost::system::error_code& /*error*/, size_t /*bytes_sent*/);

    private:
        boost::asio::io_service& mIOService;
        boost::asio::ip::udp::socket mSocket;
        boost::asio::ip::udp::endpoint mEndpoint;
        char mpData[MAX_LENGTH];
    };
}
/* End of File */
