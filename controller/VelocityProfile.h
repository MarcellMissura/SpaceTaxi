#ifndef VELOCITYPROFILE_H
#define VELOCITYPROFILE_H
#include "util/Vector.h"
#include "pml/hpm2D.h"
#include "KeyframePlayer/BangBang.h"
#include "geometry/VisibilityGraph.h"

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
