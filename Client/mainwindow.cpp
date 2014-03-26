#include "mainwindow.h"
#include "ui_mainwindow.h"


pcl::visualization::PCLVisualizer visualizer("Dont Show", false);

unsigned int i = 0;
MainWindow::MainWindow()
{
    mUi = new Ui_MainWindow;
    mUi->setupUi(this);
    mPointCloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));

//    for(int i = 0; i < 50; i++)
//    {
//        mPointCloud->push_back(pcl::PointXYZ(100.0 * float(rand()) / float(RAND_MAX),
//                                             100.0 * float(rand()) / float(RAND_MAX),
//                                             100.0 * float(rand()) / float(RAND_MAX)));
//    }

    visualizer.setShowFPS(false);
    visualizer.addPointCloud<pcl::PointXYZ>(mPointCloud);
    //mRenderer = visualizer.getRendererCollection()->GetFirstRenderer();
//    mUi->vtkWidget->GetRenderWindow()->AddRenderer(mRenderer);
    mUi->vtkWidget->SetRenderWindow(visualizer.getRenderWindow());
}

void MainWindow::StartThread()
{
    mUpdateThread.join();
    mUpdateThread.detach();
    mThreadQuitFlag = false;
    mUpdateThread = boost::thread(
                    boost::bind(&MainWindow::ShowPointCloud, this));
}

// Slot for arrival scans.
void MainWindow::ShowPointCloud()
{
    while(!mThreadQuitFlag)
    {
        mMutex.lock();
        mPointCloud->clear();

//        for(int i = 0; i < 50; i++)
//        {
//            mPointCloud->push_back(pcl::PointXYZ(100.0 * float(rand()) / float(RAND_MAX),
//                                                 100.0 * float(rand()) / float(RAND_MAX),
//                                                 100.0 * float(rand()) / float(RAND_MAX)));
//        }

        pcl_data temp;
        if(!PointQueue.empty()){
            int amt = PointQueue.size();
            for(int j=0;j<amt;j++)
            {
                if(!lock_clear || !lock_write)
                {
                    lock_read = true;
                    pcl_data buff = PointQueue[j];
                    lock_read = false;

                    for(int i = 0;i<buff.points.size();i++)
                    {
                        temp.points.push_back(buff.points[i]);
                    }
                }
            }
           mPointCloud = convertPointsToPTR(temp.points);
           i++;
        }

        std::string str = boost::lexical_cast<std::string> (i);
        visualizer.addPointCloud<pcl::PointXYZ>(mPointCloud, str);

        mMutex.unlock();
        mUi->vtkWidget->update();
        //visualizer.updateCamera();
        boost::this_thread::sleep(boost::posix_time::millisec(50));
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::convertPointsToPTR(std::vector<pcl_point> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ point;
    for(unsigned int i = 0; i < points.size(); i++)
    {

            //TODO: Convert these to cartesian
            double r = points[i].r;
            double theta = points[i].theta;
            double phi = DEG2RAD(points[i].phi);
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);

        //calculate distance & colorize
        //point.rgb = GetDistanceColor(point.x,point.y,point.z);
           // point.rgb = 999999;
        point_cloud_ptr->points.push_back (point);

    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
}



