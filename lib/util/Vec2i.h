#ifndef VEC2I_H_
#define VEC2I_H_

#include "VecNi.h"
#include <QPointF>

class Vec2i : public VecNi<2>
{

public:

    Vec2i() : VecNi<2>() {}
    Vec2i(const VecNi<2> &o) : VecNi<2>(o) {}
    Vec2i(const VecNi<1> &o, double xo) : VecNi<2>(o, xo) {}
    Vec2i(double xo, const VecNi<1> &o) : VecNi<2>(o, xo) {}
    Vec2i(const VecNi<3> &o) : VecNi<2>(o) {}
    Vec2i(const int* o) : VecNi<2>(o) {}
    Vec2i(int xo, int yo, int zo=0, int wo=0, int ao=0, int bo=0, int co=0) : VecNi<2>(xo, yo, zo, wo, ao, bo, co) {}

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
    double angleTo(const Vec2i &o) const
    {
        return fpicut(atan2(o.y, o.x) - atan2(y, x));
    }
};

#endif
