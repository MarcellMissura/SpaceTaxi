#ifndef VEC3_H_
#define VEC3_H_

#include "VecN.h"

class Vec3 : public VecN<3>
{
public:

    Vec3() : VecN<3>() {}
    Vec3(const VecN<3> &o) : VecN<3>(o) {}
    Vec3(const VecN<2> &o, double xo) : VecN<3>(o, xo) {}
    Vec3(double xo, const VecN<2> &o) : VecN<3>(xo, o) {}
    Vec3(const VecN<4> &o) : VecN<3>(o) {}
    Vec3(const double* o) : VecN<3>(o) {}
    Vec3(double o) : VecN<3>(o) {}
    Vec3(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0) : VecN<3>(xo, yo, zo, wo, ao, bo, co) {}

public:

    void operator=(const VecN<2> &o)
    {
        x = o.x;
        y = o.y;
    }

    void operator=(const VecN<3> &o)
    {
        x = o.x;
        y = o.y;
        z = o.z;
    }

    void operator=(double o)
    {
        x = o;
        y = o;
        z = o;
    }

    // Cross product.
    Vec3 operator^(const Vec3 &b) const
    {
        return crossed(b);
    }

    // Cross product.
    Vec3 crossed(const Vec3 &b) const
    {
        return Vec3(y*b.z - z*b.y,
                   z*b.x - x*b.z,
                   x*b.y - y*b.x);
    }

    Vec3 operator-(const Vec3 &o) const
    {
        Vec3 c = *this;
        c -= o;
        return c;
    }

    Vec3 operator-() const
    {
        Vec3 c = *this;
        c *= -1;
        return c;
    }

    // Rotates the x and y components of the vector by the angle.
    void rotate(double angle)
    {
        if (fabs(angle) < EPSILON)
            return;
        double c = cos(angle);
        double s = sin(angle);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    // Rotates the x and y components of the vector by the angle using a fast approximation.
    void frotate(double angle)
    {
        if (fabs(angle) < EPSILON)
            return;
        double c = fcos(angle);
        double s = fsin(angle);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }
};

#endif
