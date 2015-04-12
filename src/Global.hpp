/**
  \file Global.hpp
  \brief Global include file for global variables.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "PointStructs.hpp"
#include <sys/time.h>



static int TimeMs(void)
{
    static int isInitialized = 0;
    static struct timeval firstTime;
    struct timeval currentTime;
    long msTime;

    if (!isInitialized)
    {
        gettimeofday(&firstTime, NULL);
        isInitialized = 1;
    }
    gettimeofday(&currentTime, NULL);
    msTime =
            ((currentTime.tv_sec - firstTime.tv_sec) * 1000) +
            ((currentTime.tv_usec - firstTime.tv_usec) / 1000);

    return msTime;
}
extern std::deque<Pidar::pcl_data> gSendPoints;
extern unsigned short gLEDCount;
extern bool gStopFlag;
extern bool gISRFlag;

/* End of File */
