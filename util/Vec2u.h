#ifndef VEC2U_H_
#define VEC2U_H_

#include "VecNu.h"
#include <QPointF>
#include "globals.h"

class Vec2u : public VecNu<2>
{

public:

    Vec2u() : VecNu<2>() {}
    Vec2u(const VecNu<2> &o) : VecNu<2>(o) {}
    Vec2u(const VecNu<1> &o, double xo) : VecNu<2>(o, xo) {}
    Vec2u(double xo, const VecNu<1> &o) : VecNu<2>(o, xo) {}
    Vec2u(const VecNu<3> &o) : VecNu<2>(o) {}
    Vec2u(const uint* o) : VecNu<2>(o) {}
    Vec2u(uint xo, uint yo, uint zo=0, uint wo=0, uint ao=0, uint bo=0, uint co=0) : VecNu<2>(xo, yo, zo, wo, ao, bo, co) {}

    operator QPointF() const {return QPointF(x, y);}

public:

    void rotate(double angle)
    {
        double c = fcos(angle);
        double s = fsin(angle);
        double x_ = x;
        double y_ = y;
        x = x_*c + y_*-s;
        y = x_*s + y_*c;
    }

    // Returns the signed angle from this to the given vector.
    double angleTo(const Vec2u &o) const
    {
        return fpicut(atan2(o.y, o.x) - atan2(y, x));
    }
};

#endif
