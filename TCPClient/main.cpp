////////////////////////////////////////////////////////////////////////////////
///
/// \file main.cpp
/// \brief Main file for client application
/// Author: Jonathan Ulrich
/// Created: 2/11/13
/// Email: JonGUlrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////

#define CLIENT
#ifdef CLIENT

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
    std::cout << "Genarating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
//    for (float z(-1.0); z <= 1.0; z += 0.05)
//    {
//        for (float angle(0.0); angle <= 360.0; angle += 0.25)
//        {
//            pcl::PointXYZRGB point;
//            point.x = cosf (pcl::deg2rad(angle));
//            point.y = sinf (pcl::deg2rad(angle));
//            point.z = z;
//            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
//                  static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//            point.rgb = *reinterpret_cast<float*>(&rgb);
//            point_cloud_ptr->points.push_back (point);
//        }
//        if (z < 0.0)
//        {
//            r -= 12;
//            g += 12;
//        }
//        else
//        {
//            g -= 12;
//            b += 12;
//        }
//    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
 }

#include <fstream>
std::ifstream infile("../Pidar/sampledata.txt");
pcl_data GetSampleTXTFileData(){
    pcl_data Scan;
    double a, b, c = 0.0;
    std::string str;
    int linecnt = 0;
    std::string delimiter = ",";
    std::string val;
     while (std::getline(infile, str))
    {
         int x = 0;
         size_t pos = 0;
         while ((pos = str.find(delimiter)) != std::string::npos) {
             val = str.substr(0, pos);

             if(x==0){
                 a = atof(val.c_str());x++;
             }
             else if(x==1){
                 b = atof(val.c_str());x++;
             }
             else if(x==2){
                 c = atof(val.c_str());x=0;
             }
             str.erase(0, pos + delimiter.length());
         }

        pcl_point point;
        point.r = a;
        point.theta = b;
        point.phi = c;
        Scan.points.push_back(point);
        linecnt++;
    }
    linecnt;
    return Scan;
}

int main()
{
    try
    {

        ClientSide::setServer("192.168.1.70","10000"); //not needed unless changing values

        Render::Visual* newvisual = new Render::Visual();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = newvisual->getViewer(GetSamplePTRData());
        int x = 0;
        //bool swap = false;


        pcl_data viewcloud;

       // boost::thread bt(boost::bind(&boost::asio::io_service::getLatestCloud, &mycloud));

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));

            if(x%5==1 && x>1)
            {
                //TODO: Grab new data from server
//                if(swap == true){
//                  // viewer->updatePointCloud(GetSamplePTRData(),"Point Cloud");
//                   swap = false;
//                }
//                else
//                {
                    pcl_data mycloud;
                    //viewer->updatePointCloud(GetSampleTXTFileData(),"Point Cloud"); //local source
                    //TODO: handle bad connection/data
                    try{


                        mycloud = ClientSide::getLatestCloud();//remote source

                    for(int i=0;i<mycloud.points.size();i++)
                    {
                        pcl_point pnt;
                        pnt.r = mycloud.points[i].r;
                        pnt.phi = mycloud.points[i].phi;
                        pnt.theta = mycloud.points[i].theta;
                        viewcloud.points.push_back(pnt);
                    }
                        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
                        viewer->updatePointCloud(convertDataToPCL(viewcloud),"Point Cloud");
                    }
                    catch(std::exception e){
                        std::cout<<"Failed to receive point cloud: "<<e.what()<<std::endl;
                    }

            if(viewcloud.points.size() > 60000)
                   {
                       viewcloud.points.clear();
                   }

                    //swap = true;
//                }
               std::cout<<"Data Updated"<<std::endl;
            }

            //pcl_data received;
            //received = ClientSide::getLatestSpeed();
            //std::cout<<"Received ID: "<<received.id<<std::endl;
            //std::cout<<"Received Message: "<<received.message<<std::endl;
           // std::cout<<"Speed Reported: "<<received.speed<<std::endl<<std::endl;
            x++;
        }


//        while(1)
//        {
//            pcl_data mycloud;
//            mycloud = ClientSide::getLatestCloud();//remote source
//                boost::this_thread::sleep (boost::posix_time::milliseconds (3000));
//        }

//        int start = 0;
//        int stop = 1080;
//        //Testing while loop
//        while (!viewer->wasStopped ())
//        {
//            viewer->spinOnce (1000);
//            //boost::this_thread::sleep (boost::posix_time::milliseconds (3000));

//            pcl_data mycloud;
//            //viewer->updatePointCloud(GetSampleTXTFileData(),"Point Cloud"); //local source
//            //TODO: handle bad connection/data
//            try{
//            mycloud = ClientSide::getLatestCloud();//remote source
//                boost::this_thread::sleep (boost::posix_time::microseconds (10000));
//                viewer->updatePointCloud(convertDataToPCLPartial(mycloud,start,stop),"Point Cloud");
//                start+=0;
//                stop+=1080;
//                if(stop>10000){
//                    start = 0;
//                    stop = 1080;
//                }
//            }
//            catch(std::exception e){
//                std::cout<<"Failed to receive point cloud: "<<e.what()<<std::endl;
//            }

//        }



    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}


#endif
