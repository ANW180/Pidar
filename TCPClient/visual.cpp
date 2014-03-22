////////////////////////////////////////////////////////////////////////////////
///
/// \file visual.cpp
/// \brief Visualization class source file for client visualization of
///        point clouds.
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include "visual.hpp"

using namespace Render;
    ///////////////////////////
    //
    //  Helper Functions
    //
    //////////////////////////
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
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Point Cloud");
        viewer->setPointCloudRenderingProperties

              (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
        viewer->addCoordinateSystem(1.0);


        viewer->addText("Pidar",0,0,100,1,1,1,"Pidar",0);
        viewer->addText("Points Shown:",350,10,25,1,1,1,"Points Shown:",0);
        viewer->addText("0",525,10,20,1,1,1,"points",0);


       // 0.865668,3.3245/0.18839,0.14193,2.32075/0.34419,0.184331,-1.47331/-0.676125,-0.735908,-0.0359887/0.8575/832,469/73,80
        viewer->setCameraClipDistances(0.865668, 3.3245);
        viewer->setCameraPosition(0.18839,0.14193,2.32075, 0.34419,0.184331,-1.47331, -0.676125,-0.735908,-0.0359887);
        viewer->setCameraFieldOfView(0.8575);
        viewer->setSize(832, 469);
        viewer->setPosition(73, 80);

        return (viewer);
    }

    int GetDistanceColor(double x, double y, double z){
        int rgb;
        double  distance = sqrt(pow((x), 2) + pow((y), 2) + pow((z), 2));
        double scale = 0.5;
        distance *=scale;
        if(distance < 0.3)
            rgb = ((int)209) << 16 | ((int)0) << 8 | ((int)0); //red
        else if(distance > 0.3 && distance < 0.6)
            rgb = ((int)255) << 16 | ((int)102) << 8 | ((int)304); //orange
        else if(distance > 0.6 && distance < 1.0)
            rgb = ((int)255) << 16 | ((int)218) << 8 | ((int)33); //yellow
        else if(distance > 1.0 && distance < 1.5)
            rgb = ((int)51) << 16 | ((int)221) << 8 | ((int)0); //green
        else if(distance > 1.5 && distance < 2.0)
            rgb = ((int)17) << 16 | ((int)51) << 8 | ((int)204); //blue
        else if(distance > 2.0 && distance < 3.0)
            rgb = ((int)34) << 16 | ((int)0) << 8 | ((int)102); //indigo
        else
            rgb = ((int)51) << 16 | ((int)0) << 8 | ((int)68); //violet

        return rgb;

    }


    //////////////////////////
    //
    //Class Functions
    //
    //////////////////////////

    Visual::Visual()
    {
    }
    Visual::~Visual()
    {
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    Visual::getViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = rgbVis(point_cloud_ptr);
        return viewer;
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDataToPCL(pcl_data Data)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
                (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointXYZRGB point;
        for(unsigned int i = 0; i < Data.points.size(); i++)
        {

                //TODO: Convert these to cartesian
                double r = Data.points[i].r;
                double theta = Data.points[i].theta;
                double phi = DEG2RAD(Data.points[i].phi);
                point.x = r * sin(theta) * cos(phi);
                point.y = r * sin(theta) * sin(phi);
                point.z = r * cos(theta);

            //calculate distance & colorize
            point.rgb = GetDistanceColor(point.x,point.y,point.z);
            point_cloud_ptr->points.push_back (point);

        }
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
        point_cloud_ptr->height = 1;
        return point_cloud_ptr;
    }
