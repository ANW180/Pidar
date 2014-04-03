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
    std::cout<< "Found Camera"<<std::endl;
    return true;

}
IplImage* ImageCapture::getStoredImage()
{
    return mImage;
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

IplImage* ImageCapture::ObtainImage()
{

    mImage =  cvQueryFrame(  mCapture );
    return mImage;
}

unsigned char* ImageCapture::getColorData(IplImage* image){
    image = ObtainImage();

    // get the pointer to the image buffer
    unsigned char *data= reinterpret_cast<unsigned char *>
                                         (image->imageData);

    return data;

}

unsigned char* ImageCapture::getPixelData(IplImage* image, int x, int y){

    int nl= image->height; // number of lines
    // total number of element per line
    int nc= image->width * image->nChannels;
    int step= image->widthStep; // effective width

    unsigned char *data = new unsigned char [3];

    //Check if coordinates are inbounds
    if((x > step || y>nl) || (x<0) || (y<0))
    {
        data[0] = '-';
        data[1] = '-';
        data[2] = '-';
        return data; //empty return
    }

    //Grab pixel location
    int location = (x*image->nChannels) + (y*step);
    unsigned char *imagedata= reinterpret_cast<unsigned char *>
                                               (image->imageData);
    //Grab RGB data
    data[0] = imagedata[location];  //B
    data[1] = imagedata[location+1];//G
    data[2] = imagedata[location+2];//R

    return data;
//    unsigned char pixelB, pixelG, pixelR;
//    for (int i=1; i<nl; i++) {
//          for (int j=0; j<nc; j+= image->nChannels) {

//          // process each pixel ---------------------

//                data[j]= data[j];
//                data[j+1]= data[j+1];
//                data[j+2]= data[j+2];

//                pixelB = data[j];
//                pixelG = data[j+1];
//                pixelR = data[j+2];

//          // end of pixel processing ----------------

//          } // end of line
//             data+= step;
//    }

}







