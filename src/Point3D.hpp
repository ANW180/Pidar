/**
  \file Point3D.hpp
  \brief Point3D class for storing LIDAR points.
  \author Jonathan Ulrich (jongulrich@gmail.com)
  \author Andrew Watson (watsontandrew@gmail.com)
  \date 2014
*/
#pragma once
#include <vector>


namespace Pidar
{
    /**
     * @brief The Point3D class is a basic template for
     * storing 3D points.
     */
    class Point3D
    {
    public:
        typedef std::vector<Point3D> List;
        Point3D()
        {
            mX = mY = mZ = mIntensity = 0.0;
        }
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
