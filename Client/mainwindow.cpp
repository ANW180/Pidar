#include "mainwindow.h"
#include "ui_mainwindow.h"


pcl::visualization::PCLVisualizer visualizer("Dont Show", false);
std::string address = "192.168.1.71";
std::string port = "10001";
int normalvalues = 0;
int GetRGBValue(double position, int scaleposition)
{
    double maxdistance = 30;
    double scale = scaleposition/maxdistance;
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
    double m = nmax * position;
    int n= int(m); // integer of m
    double f = m - n;  // fraction of m
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

int MainWindow::GetCameraRGBValue(IplImage* image, double x, double y){

//    if(ally>image->height)
//    {
//        ally=0;
//    }
//    else
//    {
//        ally++;
//    }
//    if(allx>image->widthStep)
//    {
//        allx = 0;
//    }
//    else
//    {
//        allx++;
//    }
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
 //   mImage.CaptureImage();

    mUi = new Ui_MainWindow;
    mPauseScan = false;
    mPointCount = 0;
    mUi->setupUi(this);
    mPointCloud = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                   (new pcl::PointCloud<pcl::PointXYZRGB>));
    visualizer.setShowFPS(false);
    visualizer.setCameraPosition(0,-1,-14,-16,-96,-17);
    visualizer.addPointCloud<pcl::PointXYZRGB>(mPointCloud);
    mUi->vtkWidget->SetRenderWindow(visualizer.getRenderWindow());
}

MainWindow::~MainWindow()
{
    mThreadQuitFlag = true;
    mUpdateThread.join();
    mUpdateThread.detach();
    mWebThread.interrupt();
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

    double lastKnownAngle = -99;
    double degeesTraversed = 0;
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
                        pcl_data temp2 = mDisplayData;
                        mDisplayData.points.clear();
                        for(int i = temp2.points.size() - maxVal;
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
                if(mUi->radioFullScan->isChecked())
                {
                    pcl_data cachedData = mDisplayData;
                    mDisplayData.points.clear();
                    for(int i = 0;i<temp.points.size();i++)
                    {
                        cachedData.points.push_back(temp.points[i]);
                    }

                    int cacheSize = cachedData.points.size()-1;
                    pcl_data swap;

                    //Get Last Angle
                    if(lastKnownAngle == -99) // first time
                    {
                        lastKnownAngle = cachedData.points[cacheSize].phi;
                    }

                    for(int i = cacheSize;i>0;i--)
                    {
                        double delta = std::abs(lastKnownAngle - cachedData.points[i].phi);
                        if(delta > 350)
                        {
                            break;
                        }

                        swap.points.push_back(cachedData.points[i]); //added in reverse
                    }

                    //set last angle
                    lastKnownAngle =  cachedData.points[cacheSize].phi;

                   temp.points.clear();
                   int swapSize = swap.points.size()-1;
                   for(int i = swapSize;i>0;i--)
                   {
                       temp.points.push_back(swap.points[i]);
                   }

                }

                //Add points to display
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

    if(mUi->radioDispLive->isChecked())
    {
        //Capture data here first and use it in the for loop
    }

    pcl::PointXYZRGB point;
  //  IplImage* image = mImage.ObtainImage();
    for(unsigned int i = 0; i < points.size(); i++)
    {

            //TODO: Convert these to cartesian
            double r = points[i].r;
            double theta = points[i].theta;
            double phi = DEG2RAD(points[i].phi);
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);

            point.rgb = 16777215; //default to white
            if(mUi->radioDispRGB->isChecked())
            {
                point.rgb = GetRGBValue(r,mUi->slideScale->value());
            }
            else if(mUi->radioDispSolid->isChecked())
            {
                point.rgb = 16777215;
            }
            else if(mUi->radioDispLive->isChecked())
            {
                point.rgb = 16777215;
                //go through captured data and
                //assign RGB values
             //   point.rgb = GetCameraRGBValue(image,point.x,point.y);
                if(point.rgb == 0)
                {
                    point.rgb = GetRGBValue(r,mUi->slideScale->value());
                }
            }

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

void MainWindow::WritePointsToFile(pcl_data data, std::string filename)
{
    std::ofstream outputFile;
    outputFile.open(filename.c_str());

    for(int i = 0;i<data.points.size();i++)
    {
        outputFile << data.points[i].r << ","
                   << data.points[i].theta <<","
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
                                 "/home",tr("CSV Files (*.csv)"));

    std::string SaveLocation = qs.toUtf8().constData();
    if(SaveLocation != "")
    {
        WritePointsToFile(mDisplayData,SaveLocation);
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
        Scan.points.push_back(point);
        linecnt++;
    }
    linecnt;
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

        mDisplayData = OpenFileData(OpenLocation);
        globMutex.lock();
        mPointCloud = convertPointsToPTR(mDisplayData.points);
        mPointCount = mDisplayData.points.size();
        visualizer.updatePointCloud(mPointCloud);
        globMutex.unlock();

        mPointCloud->clear();
        QString label = QString::number(mPointCount);
        mUi->labelPointCount->setText(label);
        mMutex.unlock();
        mUi->vtkWidget->update();
    }

}

