#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <../Pidar/pointcloud.hpp>

namespace Render{
    class Visual{
    public:

        Visual();

        void show(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);
        void show(PointCloud::pcl_data);

    };

}
