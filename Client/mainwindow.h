#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
//#include <pcl/visualization/window.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <QMainWindow>
#include <boost/thread.hpp>
#include "receiver.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::udp;
enum { max_length = 1024 };

class Ui_MainWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();
    virtual void StartThread();
    virtual void ShowPointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertPointsToPTR(std::vector<pcl_point> points);

public slots:


private slots:
    void on_btnClear_clicked();

    void on_btnSetSpeed_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushPauseResume_clicked();

private:
    vtkSmartPointer<vtkRenderer> mRenderer;
    vtkSmartPointer<vtkRenderWindow> mRenderWindow;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPointCloud;
    Ui_MainWindow *mUi;
    Ui::MainWindow *ui;
    boost::thread mUpdateThread;
    boost::mutex mMutex;
    unsigned int mPointCount;
    bool mThreadQuitFlag;
    bool mPauseScan;
    unsigned int mLoopCount;
    pcl_data mDisplayData;
};
#endif // MAINWINDOW_H
