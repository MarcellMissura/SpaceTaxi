#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_
#include "Keyframe.h"
#include "lib/util/Vector.h"
#include <QPainter>

class CubicSpline
{
    Vector<Keyframe> keyframes;
    Vector<Keyframe> ctrl;

public:

    CubicSpline();
    ~CubicSpline(){}

    void clear();
    void reset();

    void addKeyframes(const Vector<Keyframe>& inputFrames);
    void addKeyframe(const Keyframe& kf);
    void addKeyframe(double t, double x=0, double v=0, double a=0);
    Vector<Keyframe>& getKeyframes();

    const Vector<Keyframe>& getControlSequence();
    Keyframe evaluateAt(double dt) const;
    void draw(QPainter& painter, const QPen& pen) const;
};

#endif // CUBICSPLINE_H_
