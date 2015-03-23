////////////////////////////////////////////////////////////////////////////////
///
/// \file point3d.hpp
/// \brief Point 3D class for storing LIDAR points.
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>

class Point3D
{
public:
    typedef std::vector<Point3D> List;
    Point3D()
    {
        mX = 0.0;
        mY = 0.0;
        mZ = 0.0;
        mIntensity = 0.0;
    }
    ~Point3D(){}
    void SetX(float val){ mX = val;}
    void SetY(float val){ mY = val;}
    void SetZ(float val){ mZ = val;}
    void SetIntensity(float val){mIntensity = val;}
    float GetX(){return mX;}
    float GetY(){return mY;}
    float GetZ(){return mZ;}
    float GetIntensity(){return mIntensity;}

protected:
    float mX;
    float mY;
    float mZ;
    float mIntensity;
};
/* End of File */
