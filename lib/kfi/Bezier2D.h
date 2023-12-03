#ifndef BEZIER2D_H_
#define BEZIER2D_H_
#include "Keyframe2D.h"
#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"
#include <QPainter>

class Bezier2D
{    
    Vector<Keyframe2D> keyframes;

    Vec2 P1, P2;

public:

    Bezier2D();
    ~Bezier2D(){}

    void clear();
    void reset();

    void setP1(const Vec2& p1);
    void setP2(const Vec2& p2);

    void addKeyframe(double t, double x=0, double vx=0, double y=0, double vy=0);
    void addKeyframe(const Keyframe2D &kf);
    void addKeyframes(const Vector<Keyframe2D> &kfs);

    double getTotalTime() const;
    Keyframe2D evaluateAt(double dt) const;
    void draw(QPainter* painter, const QPen& pen) const;
};

#endif
