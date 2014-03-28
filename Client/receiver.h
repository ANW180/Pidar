#ifndef RECEIVER_H
#define RECEIVER_H

////////////////////////////////////////////////////////////////////////////////
///
/// \file receiver.h
/// \brief C data sturctures for packaging of data in UDP server
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <deque>
#include <boost/thread.hpp>

typedef struct pcl_point
{
    double r;
    double theta;
    double phi;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & r;
        ar & theta;
        ar & phi;
    }
}pcl_point;


typedef struct pcl_data
{
    int id;
    int scancount;
    double speed;
    std::vector<pcl_point> points;
    std::string message;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & scancount;
        ar & speed;
        ar & id;
        ar & points;
        ar & message;
    }
}pcl_data;


typedef struct pcl_commands
{
    int cmd;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & cmd;
    }
}pcl_commands;

//Globally Accessible Scan. This should always be complete.
extern std::deque<pcl_data> PointQueue;
extern bool lock_write;
extern bool lock_clear;
extern bool lock_read;
extern boost::mutex globMutex;


#endif // RECEIVER_H
