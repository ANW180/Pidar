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
    bool IsCameraValid();
    int DisplayImage();
    IplImage* ObtainImage();
    unsigned char* getColorData(IplImage* image);
    unsigned char* getPixelData(IplImage* image, int x, int y);
    IplImage* getStoredImage();
private:
    IplImage* mImage;
    CvCapture* mCapture;
    boost::mutex mMutex;
};

#endif // IMAGECAPTURE_H
