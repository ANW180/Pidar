#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <string>
#include <vector>
#include<pointcloud.hpp>

namespace pointcloud_connection {

/// Structure to hold information about a single stock.
struct pcl_data
{
  int id;
  double r[1082];
  double theta[1082];
  double phi[1082];
  std::string message;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & id;
    ar & r;
    ar & theta;
    ar & phi;
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

class Construction{
public:
    std::vector<pcl_data> CompleteScan;
    Construction();
    void addtoScan(pcl_data singlescan);
    std::vector<pcl_data> getLast();


protected:
    std::vector<pcl_data> IncompleteScan;

};


} //

#endif // SERIALIZATION_POINTCLOUD_HPP
