#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <../Pidar/pointcloud.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

namespace Render{
    class Visual{
    public:

        Visual();

        boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

        void getViewer(pcl_data);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDataToPCL(pcl_data Data);

    };

}
