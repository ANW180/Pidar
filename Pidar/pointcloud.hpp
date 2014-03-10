#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <string>
#include <vector>
#include <opencv2/core/types_c.h>

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
  int scancount;
  std::vector<pcl_point> points;
  std::string message;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & scancount;
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


using namespace PointCloud;
class Construction{
public:

    Construction();
    pcl_data addtoScan(pcl_data incomplete, std::vector<CvPoint3D64f> laserscan,
                       double currentMotorPosition, double previousMotorPosition);
    pcl_data getLast();
    pcl_data getIncompleteScan();
    void setIncompleteScan(pcl_data data);
    void setCompleteScan(pcl_data data);
    pcl_data getCompleteScan();
    void clearIncompleteScan();


protected:
    pcl_data IncompleteScan;
    pcl_data CompleteScan;

};


} //

#endif // SERIALIZATION_POINTCLOUD_HPP
