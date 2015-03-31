/**
  \file Global.hpp
  \brief Global include file for global variables.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#pragma once
#include "Control.hpp"
#include "PointStructs.hpp"


extern std::deque<Pidar::pcl_data> gSendPoints;
extern int gMotorSpeed;
extern unsigned short gLEDCount;
extern bool gStopFlag;
extern bool lock_write;
extern bool lock_clear;
extern bool lock_read;
extern bool gFoundUpdate;
extern bool gISRFlag;

/* End of File */
