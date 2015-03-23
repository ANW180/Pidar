////////////////////////////////////////////////////////////////////////////////
///
/// \file pointstructs.hpp
/// \brief C data sturctures for packaging of data in TCP server
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>
#include <string>
#include <deque>

typedef struct pcl_point
{
    float r = 0.0;
    float theta = 0.0;
    float phi = 0.0;
    float intensity = 0.0;
    float newScan = 0.0;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & r;
        ar & theta;
        ar & phi;
        ar & intensity;
        ar & newScan;
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
extern std::deque<pcl_data> gSendPoints;
extern bool lock_write;
extern bool lock_clear;
extern bool lock_read;
extern int gMotorSpeed;
extern bool gFoundUpdate;
extern bool gISRFlag;

/* End of File */
