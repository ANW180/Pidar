/**
  \file Global.hpp
  \brief Global include file for global variables.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include "PointStructs.hpp"

extern std::deque<Pidar::pcl_data> gSendPoints;
extern unsigned short gLEDCount;
extern bool gStopFlag;
extern bool gISRFlag;

/* End of File */
