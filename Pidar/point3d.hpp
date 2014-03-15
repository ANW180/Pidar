#ifndef POINT3D_H
#define POINT3D_H
#include <vector>

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
   void SetX(double val){ mX = val;}
   void SetY(double val){ mX = val;}
   void SetZ(double val){ mX = val;}
   double GetX(){return mX;}
   double GetY(){return mY;}
   double GetZ(){return mZ;}

protected:
    double mX;
    double mY;
    double mZ;
};

#endif // POINT3D_H
