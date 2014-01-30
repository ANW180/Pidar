////////////////////////////////////////////////////////////////////////////////
///
/// \file dynamixel.hpp
/// \brief Interface for connecting to Dynamixel Motors.
/// Author: Jonathan Ulrich
/// Created: 1/28/13
/// Email: jongulrich@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef DYNAMIXEL_HPP
#define DYNAMIXEL_HPP
#include <boost/thread.hpp>
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
    /// \class Callback
    /// \brief Used to generate callbacks for subscribers to servo data.
    ///        Overload this callback and functions to recieve updated laser
    ///        scans.
    ///
    ////////////////////////////////////////////////////////////////////////////
    class Callback
    {
    public:
        Callback() {}
        virtual ~Callback() {}
        typedef std::set<Callback*> Set;
        /** Function called when new data becomes available from the laser,
            \param[in] Position data. */
        virtual void ProcessServoData(const double pos) = 0;
    };
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

#endif // DYNAMIXEL_HPP
/* End of File */
