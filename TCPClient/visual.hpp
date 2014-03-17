////////////////////////////////////////////////////////////////////////////////
///
/// \file visual.hpp
/// \brief Main file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef VISUAL_HPP
#define VISUAL_HPP
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "../Pidar/pointstructs.hpp"
#include <boost/bind.hpp>
#include <boost/asio.hpp>

namespace Render
{
    class Visual
    {
    public:
        Visual();
        ~Visual();
        boost::shared_ptr
        <pcl::visualization::PCLVisualizer> getViewer
        (pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

        void getViewer(pcl_data);
        pcl::PointCloud
        <pcl::PointXYZRGB>::Ptr convertDataToPCL(pcl_data Data);
        pcl::PointCloud
        <pcl::PointXYZRGB>::Ptr convertDataToPCLPartial(pcl_data Data,
                                                        unsigned int start,
                                                        unsigned int stop);
    };
}
#endif
