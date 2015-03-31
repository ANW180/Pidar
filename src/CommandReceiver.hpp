////////////////////////////////////////////////////////////////////////////////
///
/// \file command_receiver.hpp
/// \brief Control the Pidar
/// Author: Andrew Watson
/// Created: 3/26/15
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "point_structs.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define MAX_RPM 60
#define MAX_LENGTH 1024


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
