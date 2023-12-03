#ifndef VELOCITYPROFILE_H
#define VELOCITYPROFILE_H
#include "lib/util/Vector.h"
#include "lib/kfi/Keyframe2D.h"
#include "lib/kfi/BangBang.h"

class LinearBangBangProfile2D
{
public:

    Vector<Keyframe2D> keyframes;
    Vector<Keyframe2D> ctrl;

    double VU;
    double VL;
    double A;

public:

    LinearBangBangProfile2D();
    ~LinearBangBangProfile2D(){}

    Hpm2D getWaypoint(const Hpm2D& currentState, const Vector<Vec2> &path, double time) const;
    double getTotalTime(const Hpm2D& currentState, const Vector<Vec2> &path) const;

};

#endif // VELOCITYPROFILE_H
