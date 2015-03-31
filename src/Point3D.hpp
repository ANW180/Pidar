/**
  \file Point3D.hpp
  \brief Point3D class for storing LIDAR points.
  \authors Jonathan Ulrich (jongulrich@gmail.com), Andrew Watson (watsontandrew@gmail.com
  \date 2014
*/
#pragma once
#include <vector>


namespace Pidar
{
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
}
/* End of File */
