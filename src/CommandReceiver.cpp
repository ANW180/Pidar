#include "command_receiver.hpp"

using boost::asio::ip::udp;


CommandReceiver::CommandReceiver(boost::asio::io_service& ioService, short port)
    : mIOService(ioService),
      mSocket(ioService, udp::endpoint(udp::v4(), port))
{
    mSocket.async_receive_from(
                boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                boost::bind(&CommandReceiver::HandleReceieve, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}


std::string CommandReceiver::HandleCommands(char data[], size_t bytes_received)
{
    //Print out received data (debugging)
    for(unsigned int i = 0; i < bytes_received; i++)
    {
        std::cout << data[i];
    }
    std::cout << std::endl;

    //Initialize to INVALID command if nothing is found
    std::string response = "INVALID";
    //if Change Speed command found, update speed
    if((data[0] == 'C' && data[1] == 'S') &&
            (isdigit(data[2]) && isdigit(data[3])))
    {
        //..[C][S][4][5][]... = 45
        int spd = boost::lexical_cast<int>(data[2]) * 10;
        spd += boost::lexical_cast<int>(data[3]);
        //Do not set over max speed
        if(spd > MAX_RPM)
        {
            spd = MAX_RPM;
        }
        //Asssign max speed and set change flag to true
        gMotorSpeed = spd;
        gFoundUpdate = true;
        response = "OK";
    }
    //Get Speed Requested
    if(data[0]=='G' && data[1]=='S')
    {
        //return motor speed
        response = "SPEED=";
        response.append(boost::lexical_cast<std::string>(gMotorSpeed));
    }
    std::cout<<"Returning response: "<<response<<std::endl;
    return response;
}


void CommandReceiver::HandleReceieve(const boost::system::error_code& error,
                                     size_t bytesReceived)
{
    if(!error && bytesReceived > 0)
    {
        std::string response = "BAD";
        if(bytesReceived > 3)
        {
            response = HandleCommands(mpData, bytesReceived);
        }

        //TODO: Write responses to send back here
        mSocket.async_send_to(
                    boost::asio::buffer(response.c_str(),
                                        response.size()),
                    mEndpoint,
                    boost::bind(
                        &CommandReceiver::HandleSend,
                        this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        mSocket.async_receive_from(
                    boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                    boost::bind(&CommandReceiver::HandleReceieve,
                                this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
    }
}


void CommandReceiver::HandleSend(const boost::system::error_code& /*error*/,
                                 size_t /*bytes_sent*/)
{
    mSocket.async_receive_from(
                boost::asio::buffer(mpData, MAX_LENGTH), mEndpoint,
                boost::bind(&CommandReceiver::HandleReceieve, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}
