/**
  \file CommandReceiver.hpp
  \brief UDP interface for controlling the Pidar parameters.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "Dynamixel.hpp"
#include "PointStructs.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define MAX_LENGTH 1024


namespace Pidar
{
    /**
     * @brief The CommandReceiver class implements boost UDP services to handle
     * receiving commands to control the Pidar.
     */
    class CommandReceiver
    {
    public:
        /**
         * @brief CommandReceiver Constructor
         * @param ioService
         * @param port
         */
        CommandReceiver(boost::asio::io_service& ioService, short port);
        /**
         * @brief HandleCommands Handles UDP packet interpretation.
         * @param data
         * @param bytesReceived
         * @return
         */
        std::string HandleCommands(char data[], size_t bytesReceived);
        /**
         * @brief HandleReceieve
         * @param error
         * @param bytesReceived
         */
        void HandleReceieve(const boost::system::error_code& error,
                            size_t bytesReceived);
        /**
         * @brief HandleSend
         */
        void HandleSend(const boost::system::error_code& /*error*/,
                        size_t /*bytes_sent*/);

    private:
        boost::asio::io_service& mIOService;
        boost::asio::ip::udp::socket mSocket;
        boost::asio::ip::udp::endpoint mEndpoint;
        char mpData[MAX_LENGTH];
    };
}
/* End of File */
