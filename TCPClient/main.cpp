////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "../Pidar/connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include <client.cpp>
#include <visual.cpp>
#include "../Pidar/pointstructs.hpp"


    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetSamplePTRData(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
 }

int main()
{
    try
    {
        pcl_data x;
        x.id = 0;
        pcl_point y;
        y.r =0.0;
        y.theta = 0.0;
        y.phi = 0.0;
        x.points.push_back(y);
        SendPoints.push_back(x);

        boost::asio::io_service io_service;
        receiver r(io_service,
                   boost::asio::ip::address::from_string("0.0.0.0"),
                   boost::asio::ip::address::from_string("239.255.0.1"));
        boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service));


        Render::Visual* newvisual = new Render::Visual();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = newvisual->getViewer(GetSamplePTRData());

        pcl_data viewing;
        while (!viewer->wasStopped ())
        {



            if(!SendPoints.empty())
            {

                int amt = SendPoints.size();
                for(int j=0;j<amt;j++)
                {
                    if(!lock_clear || !lock_write)
                    {
                        lock_read = true;
                        pcl_data buff = SendPoints[j];
                        lock_read = false;


                        for(int i = 0;i<buff.points.size();i++)
                        {
                            viewing.points.push_back(buff.points[i]);
                        }
                    }

                }
                std::cout<<"Added "<<amt<<" Points";


                std::cout<<"Updating Cloud"<<std::endl;
                pcl_data viewcloud;
               // viewer->updatePointCloud(convertDataToPCL(SendPoints.front()),"Point Cloud"); //1 scan at a time shown
                viewer->updatePointCloud(convertDataToPCL(viewing),"Point Cloud");
                SendPoints.pop_front();
                std::cout<<"Cloud updated"<<std::endl;
                std::cout<<"Points Shown: "<< viewing.points.size()<<std::endl;

                if(viewing.points.size()>24500)
                {
                    viewing.points.clear();
                }




                if(!lock_clear || !lock_write)
                {
                    lock_clear = true;
                    SendPoints.clear();
                    lock_clear = false;
                }




            }
            viewer->spinOnce (100);
        }




    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
