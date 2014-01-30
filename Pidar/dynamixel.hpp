////////////////////////////////////////////////////////////////////////////////
///
/// \file dynamixel.h
/// \brief Interface for connecting to Dynamixel Motors.
/// Author: Jonathan Ulrich
/// Created: 1/28/13
/// Email: jongulrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <tinyxml.h>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <time.h>
#include <wiringPi.h>
#include <dxl_hal.h>
#include <dynamixel.h>

namespace Motor
{

    ////////////////////////////////////////////////////////////////////////////
    ///
    /// \class Dynamixel
    /// \brief Dynamixel class is used to connect to the Dynamixel motors using
    ///        an FTDI serial interface.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Dynamixel
    {
    public:
        Dynamixel();
        ~Dynamixel();

    protected:

    };
}



/* End of File */
