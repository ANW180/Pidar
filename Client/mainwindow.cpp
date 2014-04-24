#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "pcl/range_image/range_image.h"
#include "pcl/visualization/range_image_visualizer.h"

void (*signal (int sig, void (*func)(int)))(int);
pcl::visualization::PCLVisualizer visualizer("Dont Show", false);
std::string address = "192.168.1.71";
std::string port = "10001";
int normalvalues = 0;
unsigned int refreshDelay = 100;

/** Range image variables */
pcl::visualization::RangeImageVisualizer rangeImageVisualizer("Range Image");
pcl::RangeImage rangeImage;
Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
pcl::PointCloud<pcl::PointXYZ> rangeImagePointCloud;

int GetRGBValue(double position, int scaleposition)
{
    if(position < 0)
    {
        position *= -1;
    }
    double maxdistance = 30;
    double scale = scaleposition / maxdistance;
    position *= scale;

    //Don't allow higher than maximum values
    if (position > maxdistance)
    {
        position = maxdistance;
    }

    //Scale values from 0 - 1
    position = position/maxdistance;

    unsigned int R, G, B;// byte
    int nmax=6;// number of color bars
    double m = nmax * position;
    int n= int(m); // integer of m
    double f = m - n;  // fraction of m
    int t=int(f*255); //255


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

    if(R<0)
        R=0;
    if(G<0)
        G=0;
    if(B<0)
        B=0;
    if(R>255)
        R=255;
    if(G>255)
        G=255;
    if(B>255)
        B=255;

  return (int(R) << 16) | (int(G) << 8) | int(B);
}

int GetSolidRGBValue(double position, int scaleposition)
{
    unsigned int R, G, B;

    double max_scale = 100;
    double normal_scale = 10;
    double normalized_scale = (scaleposition/max_scale)*20;
    position *=normalized_scale;

    //Don't allow lower than visible values
    if (position > 220)
    {
        position = position * position/200;
        if(position>239)
        {
            position = 240;
        }
    }
          R = 255-position;
          G = 255-position;
          B = 255-position;

  return (R << 16) | (G << 8) | B;
}

int MainWindow::GetCameraRGBValue(IplImage* image, double x, double y){

    //Need to normalize values

    unsigned char* BGR = mImage.getPixelData(image,(int)x,(int)y);

    int R = BGR[2];
    int G = BGR[1];
    int B = BGR[0];


    if(R == '-' && G == '-' && B == '-')
    {
        return 0;
    }
    return (R << 16) | (G << 8) | B;
}

MainWindow::MainWindow()
{
    //Start Webcam Feed
   // mImage.CaptureImage();

    mUi = new Ui_MainWindow;
    mPauseScan = false;
    mPointCount = 0;
    mUi->setupUi(this);
    mPointCloud = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                   (new pcl::PointCloud<pcl::PointXYZRGB>));
    visualizer.setShowFPS(false);
    visualizer.setCameraPosition(0,-1,-14,-16,-96,-17);
    visualizer.addPointCloud<pcl::PointXYZRGB>(mPointCloud,"default");

    //create a rotating set of 10 point clouds
    //Helps avoid some crashing
    for(int i = 0;i<10;i++)
    {
        std::string name = boost::lexical_cast<std::string>(i);
        visualizer.addPointCloud<pcl::PointXYZRGB>(mPointCloud,name);
    }

    mUi->vtkWidget->SetRenderWindow(visualizer.getRenderWindow());
}

void MainWindow::shutdown()
{
    mThreadQuitFlag = true;
    mUpdateThread.join();
    mUpdateThread.detach();
    mWebThread.interrupt();
    QCoreApplication::instance()->exit();
}

MainWindow::~MainWindow()
{
    shutdown();
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
    int id = 1;
    while(!mThreadQuitFlag)
    {
        if(rangeImagePointCloud.size() > 0)
        {
            scene_sensor_pose = visualizer.getViewerPose();
//            scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(rangeImagePointCloud.sensor_origin_[0],
//                                                                     rangeImagePointCloud.sensor_origin_[1],
//                                                                     rangeImagePointCloud.sensor_origin_[2])) *
//                                                Eigen::Affine3f(rangeImagePointCloud.sensor_orientation_);
            rangeImage.createFromPointCloud(rangeImagePointCloud,
                                            0.0044f,    // ~0.25 radians in deg
                                            0.0044f,
                                            pcl::deg2rad(120.0f),
                                            pcl::deg2rad(75.0f),
                                            scene_sensor_pose,
                                            pcl::RangeImage::CAMERA_FRAME,
                                            0.0f,
                                            0.0f,
                                            1);
            rangeImageVisualizer.showRangeImage(rangeImage);
        }
        if(!mPauseScan)
        {
            boost::mutex::scoped_lock lock(mMutex);

            if(lock)
            {
                //mPointCloud->clear();
                pcl_data temp;


                if(!PointQueue.empty())
                {

                    //Get points from queue
                    {
                        boost::mutex::scoped_lock queuelock(globMutex);
                        if(queuelock)
                        {
                            int amt = PointQueue.size();
                            for(int j=0;j<amt;j++)
                            {
                                pcl_data buff = PointQueue[j];
                                for(int i = 0; i<buff.points.size(); i++)
                                {
                                    temp.points.push_back(buff.points[i]);
                                }
                            }
                        }
                        else
                            std::cout<<"failed to lock queue read"<<std::endl;
                    }


                    //TODO: Seperate into function
                    //Scan Methods
                    if(mUi->radioClearing->isChecked())
                    {
                        int maxVal = 10000;
                        maxVal = (int) mUi->spinClearing->value();
                        if(mDisplayData.points.size() >= maxVal)
                        {
                            pcl_data temp2 = mDisplayData;
                            mDisplayData.points.clear();
                            for(int i = temp2.points.size() - (maxVal-temp.points.size());
                                i < temp2.points.size();
                                i++)
                            {
                                mDisplayData.points.push_back(temp2.points[i]);
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

                    //Add points to display
                    for(int j =0;j<temp.points.size();j++)
                    {
                        if(temp.points[j].r > 0.001)
                        {
                            mDisplayData.points.push_back(temp.points[j]);
                        }
                    }

                    //Clear Queue
                    {
                        boost::mutex::scoped_lock queuelock(globMutex);
                        if(queuelock)
                         PointQueue.clear();
                        else
                            std::cout<<"failed to lock queue clear"<<std::endl;
                    }

                    double closestPoint = 1;
                    double furthestPoint = 1;

                    mPointCloud = convertPointsToPTR(mDisplayData.points,
                                                     closestPoint,furthestPoint);


                    mPointCount = mPointCloud->points.size();

                    mUi->labelFurthestPoint->setText(QString::number(furthestPoint));
                    mUi->labelClosestPoint->setText(QString::number(closestPoint));

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr BlankPTR =
                                                (pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                                                         (new pcl::PointCloud<pcl::PointXYZRGB>));

                    //Display clouds labled "1" - "9" in a rotating manner
                    // This is strictly a workaround method due to segfaults
                    if(id<9)
                    {
                        if(id==1)
                        {
                            visualizer.updatePointCloud(BlankPTR,"9");
                        }
                        std::string oldname = boost::lexical_cast<std::string>(id-1);
                        visualizer.updatePointCloud(BlankPTR,oldname);
                        boost::this_thread::sleep(boost::posix_time::millisec(5));
                        std::string name = boost::lexical_cast<std::string>(id);
                        visualizer.updatePointCloud(mPointCloud,name);
                        id++;
                    }
                    else
                    {
                        visualizer.updatePointCloud(BlankPTR,"8");
                        visualizer.updatePointCloud(mPointCloud,"9");
                        id = 1;
                    }
                    boost::this_thread::sleep(boost::posix_time::millisec(5));
                    mPointCloud->clear();
            }
            QString label = QString::number(mPointCount);
            mUi->labelPointCount->setText(label);
            mUi->labelPointCount_2->setText(label);

            mUi->vtkWidget->update();
            boost::this_thread::sleep(boost::posix_time::millisec(refreshDelay));
            }
            else
            {
                std::cout<<"Failed to lock"<<std::endl;
            }
        }
        else
        {
            boost::mutex::scoped_lock queuelock(globMutex);
            if(queuelock)
             PointQueue.clear();
        }
    }
}

//Compatible without maximum values
pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::convertPointsToPTR(std::vector<pcl_point> points)
{
    double closestPoint = 1;
    double furthestPoint = 1;
    return convertPointsToPTR(points,closestPoint,furthestPoint);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::convertPointsToPTR
                                (std::vector<pcl_point> points, double &closestPoint,
                                 double &furthestPoint)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::PointXYZRGB point;

    float maximumvalue_x = 1;
    float minimumvalue_x = 0;
    float maximumvalue_y = 1;
    float minimumvalue_y = 0;
    float maximumvalue_z = 1;
    float minimumvalue_z = 9999999;
    float maximumvalue_r = 0;
    float minimumvalue_r = 9999999;

    float min_allowed = 0.1;

    for(unsigned int i = 0; i < points.size(); i++)
    {
        float r = points[i].r;
        float theta = points[i].theta;
        float offset = float(mUi->spinRotationOffset->value());
        float phi = DEG2RAD(fmod(points[i].phi+offset,360.0));


        float curr_x = r * sin(theta) * cos(phi);
        float curr_y = r * sin(theta) * sin(phi);
        float curr_z = r * cos(theta);

        if(maximumvalue_x < std::abs(curr_x))
            maximumvalue_x = std::abs(curr_x);

        if(minimumvalue_x > std::abs(curr_x) && std::abs(curr_x) > min_allowed)
            minimumvalue_x = std::abs(curr_x);

        if(maximumvalue_y < std::abs(curr_y))
            maximumvalue_y = std::abs(curr_y);

        if(minimumvalue_y > std::abs(curr_y) && std::abs(curr_y) > min_allowed)
            minimumvalue_y = std::abs(curr_y);

        if(maximumvalue_z < std::abs(curr_z))
            maximumvalue_z = std::abs(curr_z);

        if(minimumvalue_z>std::abs(curr_z) && std::abs(curr_z) > min_allowed)
            minimumvalue_z = std::abs(curr_z);


    }

    maximumvalue_x *= 0.001 + float(mUi->horizontalSlider_3->value()/ 100.0);
    maximumvalue_y *= 0.001 + float(mUi->horizontalSlider_4->value()/ 100.0);
    maximumvalue_z *= 0.001 + float(mUi->horizontalSlider_2->value()/ 100.0);



    for(unsigned int i = 0; i < points.size(); i++)
    {

            //TODO: Convert these to cartesian
            float r = points[i].r;
            float theta = points[i].theta;
            //double phi = DEG2RAD(points[i].phi);
            float offset = float(mUi->spinRotationOffset->value());
            float phi = DEG2RAD(fmod(points[i].phi+offset,360.0)); //offset
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);

            if(point.x > maximumvalue_x || point.x < (-1.0*maximumvalue_x))
                continue;
            if(point.y > maximumvalue_y || point.y < (-1.0*maximumvalue_y))
                continue;
            if(point.z > maximumvalue_z || point.z < (-1.0*maximumvalue_z))
                continue;


            if(maximumvalue_r<std::abs(r))
                maximumvalue_r = std::abs(r);

            if(minimumvalue_r>std::abs(r) && std::abs(r) > min_allowed)
                minimumvalue_r = std::abs(r);

            point.rgb = 16777215; //default to white
            if(mUi->radioDispRGB->isChecked())
            {
                point.rgb = boost::lexical_cast<float>( GetRGBValue(r,mUi->slideScale->value()) );
            }

            else if(mUi->radioDispSolid->isChecked())
            {
                //point.rgb = 16777215;
                point.rgb = boost::lexical_cast<float>( GetSolidRGBValue(r,mUi->slideScale->value()) );
            }

            if(point.rgb<0 || point.rgb > 16777215)
            {
                point.rgb = boost::lexical_cast<float>(16777215);
            }

            float proximity_max = std::abs(r-maximumvalue_r);
            float proximity_min = std::abs(r-minimumvalue_r);

            if(mUi->checkHighlight->isChecked())
            {
                if(proximity_max<0.20)
                {
                    point.rgb = boost::lexical_cast<float>(13162495);
                }

                if(proximity_min<0.20)
                {
                    point.rgb = boost::lexical_cast<float>(65535);
                }
            }



        point_cloud_ptr->points.push_back (point);

    }

    furthestPoint = maximumvalue_r;
    closestPoint = minimumvalue_r;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudPtr;


    // Filtering (with 350,000 point cap)
    if(!mUi->radioShowAllPoints->isChecked() && point_cloud_ptr->points.size()<350000)
    {
        tempCloudPtr = point_cloud_ptr;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (tempCloudPtr);
        sor.setMeanK (mUi->spinNeighbors->value());
        sor.setStddevMulThresh (mUi->spinSTDDEV->value());
        if(mUi->radioShowOnlyOutliers->isChecked())
        {
            sor.setNegative(true);
        }
        sor.filter (*point_cloud_ptr);
    }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
}

void MainWindow::on_btnClear_clicked()
{
    bool waspaused = false;
    if(!mPauseScan)
    {
    mPauseScan = true;
    }
    else
    {
        waspaused = true;
    }
    mMutex.lock();
    mDisplayData.points.clear();
    mPointCloud->clear();
    mUi->labelPointCount->setText("0");
    mUi->labelPointCount_2->setText("0");
    mUi->labelClosestPoint->setText("0");
    mUi->labelFurthestPoint->setText("0");
    mPointCount = 0;
    for(int i = 0;i<10;i++)
    {
        std::string name = boost::lexical_cast<std::string>(i);
        visualizer.updatePointCloud<pcl::PointXYZRGB>(mPointCloud,name);
    }
    mMutex.unlock();


    if(!waspaused)
    {
    mPauseScan = false;
    }


    mUi->vtkWidget->update();
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
    TogglePauseScan();
}

void MainWindow::TogglePauseScan()
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

void MainWindow::on_slideScale_valueChanged(int value)
{
    //nothing
}

void MainWindow::WritePointsToFileFromPTR(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename)
{
    std::ofstream outputFile;
    outputFile.open(filename.c_str());
    std::string delimit = ",";
    for(int i = 0;i<cloud->points.size();i++)
    {
        int colour = cloud->points[i].rgb;
        int r = (colour >> 16) & 0xff;
        int g = (colour >> 8) & 0xff;
        int b = colour & 0xff;

        outputFile << cloud->points[i].x << delimit
                   << cloud->points[i].y << delimit
                   << cloud->points[i].z << delimit
                   << r << delimit
                   << g << delimit
                   << b << endl;

    }

    outputFile.close();
   // std::cout << "Wrote "<<data->points.size() <<" points, to file: "<<filename<<std::endl;
}

void MainWindow::WritePointsToFile(pcl_data data, std::string filename)
{
    std::ofstream outputFile;
    outputFile.open(filename.c_str());
    std::string delimit = ",";
    if(filename.substr(filename.find_last_of(".") + 1) == "xyz")
    {
        delimit = " ";
    }

    for(int i = 0;i<data.points.size();i++)
    {

        outputFile << data.points[i].r << delimit
                   << data.points[i].theta << delimit
                   << data.points[i].phi << endl;
    }

    outputFile.close();
    std::cout << "Wrote "<<data.points.size() <<" points, to file: "<<filename<<std::endl;
}

void MainWindow::on_actionSave_triggered()
{
    bool resume = false;
    if(!mPauseScan)
    {
        mPauseScan = true;
        mUi->pushPauseResume->setText("Resume");
        std::cout<<"Scan Paused"<<std::endl;
        resume = true;
    }

    QString qs = QFileDialog::getSaveFileName(this,tr("Select Save Location"),
                                              "/home",tr("Raw PointCloud (*.csv);;MeshLabXYZ (*.xyz);;XYZRGB (*.xyzrgb)"));

    std::string SaveLocation = qs.toUtf8().constData();
    if(SaveLocation != "")
    {
        if(SaveLocation.substr(SaveLocation.find_last_of(".") + 1) == "xyz")
        {
            pcl_data Cartesian;
            for(unsigned int i = 0; i < mDisplayData.points.size(); i++)
            {
                pcl_point tmp;
                    //Convert these to cartesian
                    double r = mDisplayData.points[i].r;
                    double theta = mDisplayData.points[i].theta;
                    double phi = DEG2RAD(mDisplayData.points[i].phi);

                    tmp.r = r * sin(theta) * cos(phi);      //actually X
                    tmp.theta = r * sin(theta) * sin(phi);  //actually Y
                    tmp.phi = r * cos(theta);               //actually Z

                    Cartesian.points.push_back(tmp);
            }
             WritePointsToFile(Cartesian,SaveLocation);

        }
        else if(SaveLocation.substr(SaveLocation.find_last_of(".") + 1) == "xyzrgb")
        {
            pcl_data Cartesian;
            for(unsigned int i = 0; i < mDisplayData.points.size(); i++)
            {
                pcl_point tmp;
                    //Convert these to cartesian
                    double r = mDisplayData.points[i].r;
                    double theta = mDisplayData.points[i].theta;
                    double phi = DEG2RAD(mDisplayData.points[i].phi);

                    tmp.r = r * sin(theta) * cos(phi);      //actually X
                    tmp.theta = r * sin(theta) * sin(phi);  //actually Y
                    tmp.phi = r * cos(theta);               //actually Z

                    Cartesian.points.push_back(tmp);
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = convertPointsToPTR(Cartesian.points);

             WritePointsToFileFromPTR(cloud,SaveLocation);

        }
        else
        {
            WritePointsToFile(mDisplayData,SaveLocation);
        }
    }

    if(resume)
    {
        mPauseScan = false;
        mUi->pushPauseResume->setText("Pause");
        std::cout<<"Scan Resumed"<<std::endl;
        resume = true;
    }

}

pcl_data MainWindow::OpenFileData(std::string filepath)
{
    std::ifstream infile(filepath.c_str());
    pcl_data Scan;
    double a, b, c = 0.0;
    std::string str;
    int linecnt = 0;
    std::string delimiter = ",";

//    if(filepath.substr(filepath.find_last_of(".") + 1) == "xyz")
//    {
//        delimiter = " ";
//    }

    std::string val;
    while (std::getline(infile, str))
    {
         int x = 0;
         size_t pos = 0;
         //while ((pos = str.find(delimiter)) != std::string::npos)
         for(int i = 0;i<3;i++)
         {
             pos = str.find(delimiter);
             val = str.substr(0, pos);
             if(x == 0)
             {
                 a = atof(val.c_str());
                 x++;
             }
             else if(x == 1)
             {
                 b = atof(val.c_str());
                 x++;
             }
             else if(x == 2)
             {
                 c = atof(val.c_str());
                 x = 0;
             }
             str.erase(0, val.length() + delimiter.length());
         }
        pcl_point point;
        point.r = a;
        point.theta = b;
        point.phi = c;

        if(a > 0.001)
        {
         Scan.points.push_back(point);
        }

        linecnt++;
    }
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> temp;
    for(unsigned int i = 0; i < Scan.points.size(); i++)
    {
            float r = Scan.points[i].r;
            float theta = Scan.points[i].theta;
            float phi = Scan.points[i].phi;
            float offset = float(mUi->spinRotationOffset->value());
            phi = DEG2RAD(fmod(phi + (100 - offset), 360.0));
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);
        temp.push_back(point);
    }
    rangeImagePointCloud = temp;
    return Scan;
}

void MainWindow::on_actionOpen_triggered()
{
        mPauseScan = true;
        mUi->pushPauseResume->setText("Resume");
        std::cout<<"Scan Paused"<<std::endl;

    QString qs = QFileDialog::getOpenFileName(this,tr("Open CSV file"),
                                 "/home",tr("CSV Files (*.csv)"));

    std::string OpenLocation = qs.toUtf8().constData();
    if(OpenLocation!="")
    {
        for(int i = 0;i<10;i++)
        {
            std::string name = boost::lexical_cast<std::string>(i);
            visualizer.updatePointCloud<pcl::PointXYZRGB>(mPointCloud,name);
        }

        mDisplayData = OpenFileData(OpenLocation);
        mPointCloud = convertPointsToPTR(mDisplayData.points);
        mPointCount = mDisplayData.points.size();
        visualizer.updatePointCloud(mPointCloud,"0");

        mPointCloud->clear();
        QString label = QString::number(mPointCount);
        mUi->labelPointCount->setText(label);
        mUi->labelPointCount_2->setText(label);
        mMutex.unlock();
        mUi->vtkWidget->update();

        //Mesh
//        pcl::PolygonMesh mesh;
//        meshRender render;
//        mesh = render.CreateMesh(OpenFileData(OpenLocation));
//        visualizer.addPolygonMesh(mesh);
//        mUi->vtkWidget->update();

    }
}


void MainWindow::on_actionExit_triggered()
{
    shutdown();
}

void MainWindow::on_btnIPSAVE_clicked()
{
    QString qIP = mUi->txtIPADDRESS->text();
    std::string ipAddress = qIP.toStdString();
    boost::system::error_code ec;
    boost::asio::ip::address::from_string( ipAddress, ec );

    if(!ec)
    {
        address = ipAddress;
        std::cout<<"IP address set to " << address <<std::endl;
    }
    else
    {
        std::cout<<"Error entering IP address: "<<ec<<std::endl;
    }

}

void MainWindow::on_spinDelay_valueChanged(int arg1)
{
    refreshDelay = arg1;
}
