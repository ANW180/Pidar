////////////////////////////////////////////////////////////////////////////////
///
/// \file pointstructs.hpp
/// \brief C data sturctures for packaging of data in TCP server
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef POINTSTRUCT_HPP
#define POINTSTRUCT_HPP
#include <vector>
#include <string>

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
extern pcl_data PublicScan;
extern pcl_data PublicPartialScan;


#endif
