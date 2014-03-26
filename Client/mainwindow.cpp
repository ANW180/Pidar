#include "mainwindow.h"
#include "ui_mainwindow.h"


pcl::visualization::PCLVisualizer visualizer("Dont Show", false);

unsigned int i = 0;
unsigned int pointcount = 0;
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

MainWindow::~MainWindow(){
    mUpdateThread.join();
    mUpdateThread.detach();
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
           pointcount += temp.points.size();
           if(!lock_clear || !lock_write || !lock_read)
           {
                PointQueue.clear();
           }

           i++;

        }

        std::string str = boost::lexical_cast<std::string> (i);
        visualizer.addPointCloud<pcl::PointXYZ>(mPointCloud,str);
        mPointCloud->clear();
        QString label = QString::number(pointcount);
        mUi->labelPointCount->setText(label);
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

void MainWindow::on_btnClear_clicked()
{
    mMutex.lock();
    pointcount = 0;
    visualizer.removeAllPointClouds();
    mMutex.unlock();
    mUi->vtkWidget->update();
}

void MainWindow::on_btnSetSpeed_clicked()
{
    std::string address = "localhost";
    std::string port = "10001";
    boost::asio::io_service io_service;
    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), address, port);
    udp::resolver::iterator iterator = resolver.resolve(query);

    std::string sendstring = "CS";
    std::string val = "11";
    val = mUi->txtSpeed->text().toStdString();
    sendstring.append(val);
    char request[max_length];
    for(int i=0;i<sendstring.length();i++)
    {
        request[i]=sendstring[i];
    }

    size_t request_length = std::strlen(request);
    s.send_to(boost::asio::buffer(request, request_length), *iterator);
}
