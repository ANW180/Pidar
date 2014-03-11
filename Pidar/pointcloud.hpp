#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <string>
#include <vector>
#include <opencv2/core/types_c.h>
#include <../Pidar/pointstructs.hpp>

namespace PointCloud {

class Construction{
public:

    pcl_data CompleteScan;
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


};


} //

#endif // SERIALIZATION_POINTCLOUD_HPP
