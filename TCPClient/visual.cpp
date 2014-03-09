#include<visual.hpp>
using namespace Render;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis
    (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
              (new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField
              <pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties
              (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      viewer->addCoordinateSystem(1.0);
      viewer->initCameraParameters();
      return (viewer);
    }

    Visual::Visual(){
    }


    void Visual::show(){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
                (new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
        point_cloud_ptr->height = 1;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = rgbVis(point_cloud_ptr);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

    }



