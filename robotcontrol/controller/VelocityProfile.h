#ifndef VELOCITYPROFILE_H
#define VELOCITYPROFILE_H
#include "lib/util/Vector.h"
#include "lib/pml/hpm2D.h"
#include "BangBang.h"

class VelocityProfile
{
public:

    Vector<Keyframe> keyframes;
    Vector<Keyframe> ctrl;

    double VU;
    double VL;
    double A;

public:

    VelocityProfile();
    ~VelocityProfile(){}

    Hpm2D getWaypoint(const Hpm2D& currentState, const Vector<Vec2> &path, double time) const;
    double getTotalTime(const Hpm2D& currentState, const Vector<Vec2> &path) const;

};

#endif // VELOCITYPROFILE_H
