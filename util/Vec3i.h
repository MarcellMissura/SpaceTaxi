#ifndef VEC3I_H_
#define VEC3I_H_

#include "VecNi.h"

class Vec3i : public VecNi<3>
{
public:

    Vec3i() : VecNi<3>() {}
    Vec3i(const VecNi<3> &o) : VecNi<3>(o) {}
    Vec3i(const VecNi<2> &o, double xo) : VecNi<3>(o, xo) {}
    Vec3i(double xo, const VecNi<2> &o) : VecNi<3>(xo, o) {}
    Vec3i(const VecNi<4> &o) : VecNi<3>(o) {}
    Vec3i(const int* o) : VecNi<3>(o) {}
    Vec3i(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0) : VecNi<3>(xo, yo, zo, wo, ao, bo, co) {}

public:

    // Cross product.
    Vec3i operator^(const VecNi<3> &b) const
    {
        return crossed(b);
    }

    // Cross product.
    Vec3i crossed(const VecNi<3> &b) const
    {
        return Vec3i(y*b.z - z*b.y,
                   z*b.x - x*b.z,
                   x*b.y - y*b.x);
    }


    void rotate(Vec3i axis, double angle)
    {
        // Needs implementation.
    }

    // Returns the signed angle from this to the given vector.
    double angleTo(const Vec3i &o) const
    {
        // Except that it's not yet properly implemented.
        return acos(qBound(-1.0, ((*this)*o) / (norm()*o.norm()), 1.0));
    }
};

#endif
