#ifndef VEC3U_H_
#define VEC3U_H_

#include "VecNu.h"

class Vec3u : public VecNu<3>
{
public:

    Vec3u() : VecNu<3>() {}
    Vec3u(const VecNu<3> &o) : VecNu<3>(o) {}
    Vec3u(const VecNu<2> &o, uint xo) : VecNu<3>(o, xo) {}
    Vec3u(uint xo, const VecNu<2> &o) : VecNu<3>(xo, o) {}
    Vec3u(const VecNu<4> &o) : VecNu<3>(o) {}
    Vec3u(const uint* o) : VecNu<3>(o) {}
    Vec3u(uint xo, uint yo, uint zo=0, uint wo=0, uint ao=0, uint bo=0, uint co=0) : VecNu<3>(xo, yo, zo, wo, ao, bo, co) {}

public:

    // Cross product.
    Vec3u operator^(const VecNu<3> &b) const
    {
        return crossed(b);
    }

    // Cross product.
    Vec3u crossed(const VecNu<3> &b) const
    {
        return Vec3u(y*b.z - z*b.y,
                   z*b.x - x*b.z,
                   x*b.y - y*b.x);
    }

    void rotate(Vec3u axis, double angle)
    {
        // Needs implementation.
    }

    // Returns the signed angle from this to the given vector.
    double angleTo(const Vec3u &o) const
    {
        // Except that it's not yet properly implemented.
        return acos(qBound(-1.0, ((*this)*o) / (norm()*o.norm()), 1.0));
    }
};

#endif
