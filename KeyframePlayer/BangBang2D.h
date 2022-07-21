#ifndef BANGBANG2D_H_
#define BANGBANG2D_H_
#include "Keyframe2D.h"
#include "BangBang.h"
#include "util/Vector.h"
#include <QPainter>
#include <QMutex>

class BangBang2D
{
    QMutex mutex;

public:

    BangBang X;
    BangBang Y;

public:
    BangBang2D();
    ~BangBang2D(){}

    void setA(double A);
    void setV(double V);
    void setVU(double VU);
    void setVL(double VL);
    void setVU(Vec2 VU);
    void setVL(Vec2 VL);

    void clear();
    void reset();

    void addKeyframe(double dt, double x=0, double vx=0, double y=0, double vy=0);
    void addKeyframe(const Keyframe2D &kf);
    void addKeyframes(const Vector<Keyframe2D> &kfs);

    Vector<Keyframe2D> getTimedControlSequence();
    Vector<Keyframe2D> getTimeOptimalControlSequence();

    Vector<Keyframe2D> threeWayPass();

    Keyframe2D evaluateAt(const Vector<Keyframe2D>& ctrl, double dt) const;
    void draw(QPainter* painter, const Vector<Keyframe2D>& ctrl, double dt=-1) const;
};

#endif // BangBang2D_H_
