#include "mainwindow.h"
#include "ui_mainwindow.h"

pcl::visualization::PCLVisualizer visualizer("Dont Show", false);
unsigned int i = 0;
MainWindow::MainWindow()
{
    mUi = new Ui_MainWindow;
    mUi->setupUi(this);
    mPointCloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    for(int i = 0; i < 50; i++)
    {
        mPointCloud->push_back(pcl::PointXYZ(100.0 * float(rand()) / float(RAND_MAX),
                                             100.0 * float(rand()) / float(RAND_MAX),
                                             100.0 * float(rand()) / float(RAND_MAX)));
    }
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
        for(int i = 0; i < 50; i++)
        {
            mPointCloud->push_back(pcl::PointXYZ(100.0 * float(rand()) / float(RAND_MAX),
                                                 100.0 * float(rand()) / float(RAND_MAX),
                                                 100.0 * float(rand()) / float(RAND_MAX)));
        }
        i++;
        std::string str = boost::lexical_cast<std::string> (i);
        visualizer.addPointCloud<pcl::PointXYZ>(mPointCloud, str);

        mMutex.unlock();
        mUi->vtkWidget->update();
        //visualizer.updateCamera();
        boost::this_thread::sleep(boost::posix_time::millisec(50));
    }
}
