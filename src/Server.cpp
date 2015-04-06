/**
  \file Server.cpp
  \brief UDP server interface class for connecting to the Pidar.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#include "Server.hpp"
#include "Global.hpp"
#include "Control.hpp"

using namespace Pidar;

const short Server::mMulticastPort = 30001;
const int Server::mTransmissionDelayMs = 1;

Server::Server(boost::asio::io_service& ioService,
               const boost::asio::ip::address& multicastAddress)
    : mEndpoint(multicastAddress, mMulticastPort),
      mSocket(ioService, mEndpoint.protocol()),
      mDeadlineTimer(ioService)
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
#ifdef DEBUG
    std::cout << "Sending Data"
              << std::endl;
    std::cout << "on "
              << mEndpoint.address()
              << ":" << mEndpoint.port()
              << std::endl;
#endif
    mSocket.async_send_to(
                boost::asio::buffer(mPointClouds), mEndpoint,
                boost::bind(&Server::HandleSend, this,
                            boost::asio::placeholders::error));
}


void Server::HandleSend(const boost::system::error_code& error)
{
    if(!error)
    {
        mDeadlineTimer.expires_from_now
                (boost::posix_time::milliseconds(mTransmissionDelayMs));
        mDeadlineTimer.async_wait(
                    boost::bind(&Server::HandleTimeout, this,
                                boost::asio::placeholders::error));
    }
}


void Server::HandleTimeout(const boost::system::error_code& error)
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
                    boost::bind(&Server::HandleSend,
                                this,
                                boost::asio::placeholders::error));
        }
        else
        {
            HandleSend(error);
        }
    }
}
/* End of File */
