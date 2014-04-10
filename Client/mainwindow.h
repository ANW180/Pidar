#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
//#include <pcl/visualization/window.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <QMainWindow>
#include <QFileDialog>
#include <boost/thread.hpp>
#include "receiver.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include "imagecapture.h"
#include <QDialog>
#include "meshrender.h"
#include <pcl/filters/statistical_outlier_removal.h>

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
    void shutdown();
    virtual void StartThread();
    virtual void ShowPointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertPointsToPTR(std::vector<pcl_point> points);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertPointsToPTR(std::vector<pcl_point> points,
                                                              double &closestPoint, double &furthestPoint);
    int GetCameraRGBValue(IplImage* image, double x, double y);
    void TogglePauseScan();
    void WritePointsToFile(pcl_data data, std::string filename);
    pcl_data OpenFileData(std::string filepath);
    boost::thread mWebThread;

public slots:


private slots:
    void on_btnClear_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushPauseResume_clicked();

    void on_slideScale_valueChanged(int value);

    void on_actionSave_triggered();

    void on_actionOpen_triggered();

    void on_actionExit_triggered();

    void on_btnIPSAVE_clicked();

    void on_spinDelay_valueChanged(int arg1);

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
    ImageCapture mImage;
};
#endif // MAINWINDOW_H
