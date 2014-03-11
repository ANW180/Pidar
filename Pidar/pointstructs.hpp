#ifndef GLOB_HPP
#define GLOB_HPP
#include<../Pidar/pointcloud.hpp>

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

struct pcl_commands
{
  int cmd;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & cmd;
  }
};

//Globally Accessible Scan. This should always be complete.
extern pcl_data PublicScan;


#endif
