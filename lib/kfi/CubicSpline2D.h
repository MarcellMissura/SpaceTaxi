#ifndef CUBICSPLINE2D_H_
#define CUBICSPLINE2D_H_
#include "Keyframe2D.h"
#include "CubicSpline.h"
#include <QPainter>

class CubicSpline2D
{
    Vector<Keyframe2D> ctrl;

public:

    CubicSpline X;
    CubicSpline Y;

public:
    CubicSpline2D();
    ~CubicSpline2D(){}

    void clear();
    void reset();

    void addKeyframe(double t, double x=0, double vx=0, double y=0, double vy=0);
    void addKeyframe(const Keyframe2D &kf);
    void addKeyframes(const Vector<Keyframe2D> &kfs);

    Vector<Keyframe2D> getControlSequence();
    double getTotalTime() const;
    Keyframe2D evaluateAt(double dt) const;
    void draw(QPainter* painter, const QPen& pen) const;
};

#endif // CUBICSPLINE2D_H_
