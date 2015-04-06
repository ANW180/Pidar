/**
  \file CommandServer.cpp
  \brief UDP interface for controlling the Pidar parameters.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Global.hpp"
#include "CommandServer.hpp"
#include "Control.hpp"

using namespace Pidar;


using boost::asio::ip::udp;


CommandServer::CommandServer(boost::asio::io_service& ioService, short port)
    : mrIOService(ioService), mSocket(ioService, udp::endpoint(udp::v4(), port))
{
    mSocket.async_receive_from(
                boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                boost::bind(&CommandServer::HandleReceieve, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}


CommandServer::~CommandServer()
{
    if(mpData)
    {
        delete[] mpData;
    }
}


std::string CommandServer::HandleCommands(char data[], size_t bytes_received)
{
#ifdef DEBUG
    //Print out received data if debugging
    for(unsigned int i = 0; i < bytes_received; i++)
    {
        std::cout << data[i];
    }
    std::cout << std::endl;
#endif
    std::string response = "INVALID";
    // Change Speed command found
    if((data[0] == 'C' && data[1] == 'S') &&
            (isdigit(data[2]) && isdigit(data[3])))
    {
        //..[C][S][4][5][]... = 45
        int speed = boost::lexical_cast<int>(data[2]) * 10;
        speed += boost::lexical_cast<int>(data[3]);
        // Asssign speed and set change flag to true
        Control::Instance()->mpMotor->SetSpeedRpm((float)speed);
    }
    return response;
}


void CommandServer::HandleReceieve(const boost::system::error_code& error,
                                     size_t bytesReceived)
{
    if(!error && bytesReceived > 0)
    {
        std::string response = "BAD";
        if(bytesReceived > 3)
        {
            response = HandleCommands(mpData, bytesReceived);
        }
        mSocket.async_send_to(
                    boost::asio::buffer(response.c_str(),
                                        response.size()),
                    mEndpoint,
                    boost::bind(
                        &CommandServer::HandleSend,
                        this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        mSocket.async_receive_from(
                    boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                    boost::bind(&CommandServer::HandleReceieve,
                                this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
    }
}


void CommandServer::HandleSend(const boost::system::error_code& /*error*/,
                                 size_t /*bytes_sent*/)
{
    mSocket.async_receive_from(
                boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                boost::bind(&CommandServer::HandleReceieve, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}
