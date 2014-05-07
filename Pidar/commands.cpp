#include "pointstructs.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define MAX_RPM 60

using boost::asio::ip::udp;
class commandserver
{
public:
    commandserver(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      socket_(io_service, udp::endpoint(udp::v4(), port))
    {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&commandserver::handle_receive_from, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    }


    std::string handle_commands(char data[],size_t bytes_received)
    {
        //Print out received data (debugging)
        for(int i = 0; i < bytes_received; i++)
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
            int spd = boost::lexical_cast<int>(data[2])*10;
            spd += boost::lexical_cast<int>(data[3]);
            //Do not set over max speed
            if(spd > MAX_RPM)
            {
                spd = MAX_RPM;
            }
            //Asssign max speed and set change flag to true
            globMotorSpeed = spd;
            globFoundUpdate = true;
            response = "OK";
        }
        //Get Speed Requested
        if(data[0]=='G' && data[1]=='S')
        {
            //return motor speed
            response = "SPEED=";
            response.append(boost::lexical_cast<std::string>(globMotorSpeed));
        }
        std::cout<<"Returning response: "<<response<<std::endl;
        return response;
    }


  void handle_receive_from(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    if (!error && bytes_recvd > 0)
    {
        std::string response = "BAD";
        if(bytes_recvd > 3)
        {
         response = handle_commands(data_,bytes_recvd);
        }

        //TODO: Write responses to send back here
      socket_.async_send_to(
          boost::asio::buffer(response.c_str(),
                              response.size()),
                              sender_endpoint_,
                              boost::bind(
                              &commandserver::handle_send_to,
                              this,
                               boost::asio::placeholders::error,
                               boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          boost::bind(&commandserver::handle_receive_from, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
  }

  void handle_send_to(const boost::system::error_code& /*error*/,
      size_t /*bytes_sent*/)
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&commandserver::handle_receive_from, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
    boost::asio::io_service& io_service_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    enum { max_length = 1024 };
    char data_[max_length];
};
