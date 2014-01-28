////////////////////////////////////////////////////////////////////////////////
///
/// \file hokuyoexample.cpp
/// \brief Example of using the hokuyo interface class.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <hokuyo.h>

int main()
{
    Laser::Hokuyo laser;
    if(laser.LoadSettings("/home/developer/software/Group27/LidarSample/hokuyo.xml"))
    {
        if(laser.Initialize())
        {
            if(laser.StartCaptureThread())
            {
                while (1)
                {
                }
            }
        }
    }

    return 0;
}
/** End of File */
