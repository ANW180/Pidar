#ifndef IMAGECAPTURE_H
#define IMAGECAPTURE_H
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>
#include "opencv2/core/core.hpp"

class ImageCapture
{
public:
    ImageCapture();
    bool CaptureImage();
    int DisplayImage();

private:
    CvCapture* mCapture;
    boost::mutex mMutex;
};

#endif // IMAGECAPTURE_H
