////////////////////////////////////////////////////////////////////////////////
///
/// \file point3d.hpp
/// \brief Point 3D class for storing LIDAR points.
/// Author: Jonathan Ulrich, Andrew Watson
/// Created: 3/1/14
/// Email: jongulrich@gmail.com, watsontandrew@gmail.com
///
////////////////////////////////////////////////////////////////////////////////
#ifndef POINT3D_H
#define POINT3D_H

class Point3D
{
public:
    Point3D()
    {
        mX = 0.0;
        mY = 0.0;
        mZ = 0.0;
    }
    ~Point3D(){}
    void SetX(float val){ mX = val;}
    void SetY(float val){ mY = val;}
    void SetZ(float val){ mZ = val;}
    float GetX(){return mX;}
    float GetY(){return mY;}
    float GetZ(){return mZ;}

protected:
    float mX;
    float mY;
    float mZ;
};

#endif // POINT3D_H
