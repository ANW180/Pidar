#ifndef HOKUYO_H
#define HOKUYO_H
#include <boost/thread.hpp>
#include <tinyxml.h>
#include <urg_utils.h>
#include <urg_sensor.h>
#include <iostream>
#include <string>

namespace Sensor
{
    class Hokuyo
    {
    public:
        Hokuyo();
        ~Hokuyo();
        virtual bool LoadSettings(const std::string& settings);
        virtual bool Initialize();
        virtual void Shutdown();
        virtual bool IsConnected() const { return mConnectedFlag; }
        virtual bool StartCaptureThread();
        virtual void StopCaptureThread();

    protected:
        virtual bool GrabRangeData();
        void CaptureThread();

        bool mConnectedFlag;
        bool mCaptureThreadFlag;
        int mBaudRate;
        void* mpDevice;
        long* mpHokuyoScan;
        int mHokuyoScanLength;
        int mHokuyoMinStep;
        int mHokuyoMaxStep;
        std::string mSerialPort;
        TiXmlDocument* mpDocument;
        boost::thread mCaptureThread;
    };
}


#endif // HOKUYO_H
/* End of File */
