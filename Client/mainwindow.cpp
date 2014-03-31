#include "mainwindow.h"
#include "ui_mainwindow.h"


pcl::visualization::PCLVisualizer visualizer("Dont Show", false);
std::string address = "192.168.1.71";
std::string port = "10001";


int GetRGBValue(double position)
{
    double maxdistance = 50.0;
    double scale = 1.0;
    position *=scale;

    //Don't allow higher than maximum values
    if (position > maxdistance)
    {
        position = maxdistance;
    }

    //Scale values from 0 - 1
    position = position/maxdistance;

    int R, G, B;// byte
    int nmax=6;// number of color bars
    double m=nmax* position;
    int n=int(m); // integer of m
    double f=m-n;  // fraction of m
    int t=int(f*255);


    switch( n)
    {
       case 0: {
          R = 255;
          G = t;
          B = 0;
           break;
        };
       case 1: {
          R = 255 - t;
          G = 255;
          B = 0;
           break;
        };
       case 2: {
          R = 0;
          G = 255;
          B = t;
           break;
        };
       case 3: {
          R = 0;
          G = 255 - t;
          B = 255;
           break;
        };
       case 4: {
          R = t;
          G = 0;
          B = 255;
           break;
        };
       case 5: {
          R = 255;
          G = 0;
          B = 255 - t;
           break;
        };

    }; // case


  return (R << 16) | (G << 8) | B;
}

MainWindow::MainWindow()
{
    mUi = new Ui_MainWindow;
    mPauseScan = false;
    mPointCount = 0;
    mUi->setupUi(this);
    mPointCloud = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                   (new pcl::PointCloud<pcl::PointXYZRGB>));
    visualizer.setShowFPS(false);
    visualizer.addPointCloud<pcl::PointXYZRGB>(mPointCloud);
    mUi->vtkWidget->SetRenderWindow(visualizer.getRenderWindow());
}

MainWindow::~MainWindow()
{
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
        if(!mPauseScan)
        {
            mMutex.lock();
            mPointCloud->clear();
            pcl_data temp;
            if(!PointQueue.empty())
            {
                globMutex.lock();
                int amt = PointQueue.size();
                globMutex.unlock();
                for(int j=0;j<amt;j++)
                {
                    globMutex.lock();
                    pcl_data buff = PointQueue[j];
                    globMutex.unlock();
                    for(int i = 0; i<buff.points.size(); i++)
                    {
                        temp.points.push_back(buff.points[i]);
                    }
                }
                //TODO: Seperate into function
                //Scan Methods
                if(mUi->radioClearing->isChecked())
                {
                    int maxVal = 300000;
                    if(mDisplayData.points.size() > maxVal)
                    {
                        pcl_data temp = mDisplayData;
                        mDisplayData.points.clear();
                        for(int i = temp.points.size() - maxVal;
                            i < temp.points.size();
                            i++)
                        {
                            mDisplayData.points.push_back(temp.points[i]);
                        }
                    }
                }
                if(mUi->radioContinuous->isChecked())
                {
                    //do nothing
                }
                if(mUi->radioSingle->isChecked())
                {
                    mDisplayData.points.clear();
                }
                for(int j =0;j<temp.points.size();j++)
                {
                    mDisplayData.points.push_back(temp.points[j]);
                }
                globMutex.lock();
                mPointCloud = convertPointsToPTR(mDisplayData.points);
                mPointCount = mDisplayData.points.size();
                PointQueue.clear();
                visualizer.updatePointCloud(mPointCloud);
                globMutex.unlock();
            }
            mPointCloud->clear();
            QString label = QString::number(mPointCount);
            mUi->labelPointCount->setText(label);
            mMutex.unlock();
            mUi->vtkWidget->update();
            //visualizer.updateCamera();
            boost::this_thread::sleep(boost::posix_time::millisec(50));
        }
        else
        {
            globMutex.lock();
            PointQueue.clear();
            globMutex.unlock();
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::convertPointsToPTR
                                (std::vector<pcl_point> points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB point;
    for(unsigned int i = 0; i < points.size(); i++)
    {

            //TODO: Convert these to cartesian
            double r = points[i].r;
            double theta = points[i].theta;
            double phi = DEG2RAD(points[i].phi);
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);

            point.rgb = GetRGBValue(r);
        point_cloud_ptr->points.push_back (point);

    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
}

void MainWindow::on_btnClear_clicked()
{
    mMutex.lock();
    mDisplayData.points.clear();
    mPointCloud->clear();
    mUi->labelPointCount->setText("0");
    mPointCount = 0;
    visualizer.updatePointCloud(mPointCloud);
    mMutex.unlock();
    mUi->vtkWidget->update();
}

void MainWindow::on_btnSetSpeed_clicked()
{
//deleting this breaks the code

}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    boost::asio::io_service io_service;
    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), address, port);
    udp::resolver::iterator iterator = resolver.resolve(query);

    std::string sendstring = "CS";
    QString label = QString::number(value);
    mUi->labelSpeed->setText(label);

    if(value < 10)
    {
        sendstring.append("0");
    }

    sendstring.append(boost::lexical_cast<std::string>(value));

    char request[max_length];
    for(int i=0;i<sendstring.length();i++)
    {
        request[i]=sendstring[i];
    }

    size_t request_length = std::strlen(request);
    s.send_to(boost::asio::buffer(request, request_length), *iterator);
    std::cout<<"Sent command: "<<sendstring<<std::endl;
}

void MainWindow::on_pushPauseResume_clicked()
{
    if(!mPauseScan)
    {
        mPauseScan = true;
        mUi->pushPauseResume->setText("Resume");
        std::cout<<"Scan Paused"<<std::endl;
    }
    else
    {
        mPauseScan = false;
        mUi->pushPauseResume->setText("Pause");
        std::cout<<"Scan Resumed"<<std::endl;
    }

}
