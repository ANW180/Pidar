////////////////////////////////////////////////////////////////////////////////
/// \file hokuyoexample.cpp
/// \brief Example of using the hokuyo interface class.
/// Author: Andrew Watson
/// Created: 1/22/13
/// Email: watsontandrew@gmail.com
////////////////////////////////////////////////////////////////////////////////
#include <hokuyo.h>

int main()
{
    Sensor::Hokuyo laser;
    if(laser.LoadSettings("hokuyo.xml"))
    {
        if(laser.Initialize())
        {
            while (1)
            {

            }
        }
    }

    return 0;
}
/** End of File */
