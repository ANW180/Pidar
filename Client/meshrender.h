#ifndef MESHRENDER_H
#define MESHRENDER_H
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

#include "receiver.h"

using namespace pcl;

class meshRender
{
public:
    meshRender();
    PolygonMesh CreateMesh(pcl_data data);

    private:
    //pcl_data mPointCloud;
    PolygonMesh mMesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointsToPTR(std::vector<pcl_point> points);
};

#endif // MESHRENDER_H
