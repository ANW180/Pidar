#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <string>

namespace pointcloud_connection {

/// Structure to hold information about a single stock.
struct pcl_data
{
  int id;
  double x[1080];
  double y[1080];
  double z[1080];

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & id;
    ar & x;
    ar & y;
    ar & z;
  }
};

struct pcl_commands
{
  int cmd;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & cmd;
  }
};

} //

#endif // SERIALIZATION_POINTCLOUD_HPP
