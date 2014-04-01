#include "imagecapture.h"

ImageCapture::ImageCapture()
{

}

bool ImageCapture::CaptureImage(){

    mMutex.lock();
    mCapture = cvCaptureFromCAM( CV_CAP_ANY ); //0=default, -1=any camera, 1..99=your camera
    mMutex.unlock();

    if(!mCapture)
    {
        std::cout << "No camera detected" << std::endl;
        return false;
    }
    return true;
}

int ImageCapture::DisplayImage()
{
    cv::Mat frame, frameCopy, image;
    cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

    if( mCapture )
    {
    std::cout << "In capture ..." << std::endl;
    for(;;)
    {
    IplImage* iplImg = cvQueryFrame( mCapture );
    frame = iplImg;

    if( frame.empty() )
    break;
    if( iplImg->origin == IPL_ORIGIN_TL )
    frame.copyTo( frameCopy );
    else
    flip( frame, frameCopy, 0 );

    cvShowImage( "result", iplImg );

    if( cv::waitKey( 10 ) >= 0 )
    break;
    }
    // waitKey(0);
    }

    cvReleaseCapture( &mCapture );
    cvDestroyWindow( "result" );

    return 0;
}
