#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <string>
#include <vector>

namespace PointCloud {

struct pcl_point
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
};

/// Structure to hold information about a single stock.
struct pcl_data
{
  int id;
  std::vector<pcl_point> points;
  std::string message;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & id;
    ar & points;
    ar & message;
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
