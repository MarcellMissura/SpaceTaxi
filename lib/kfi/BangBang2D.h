#ifndef BANGBANG2D_H_
#define BANGBANG2D_H_
#include "Keyframe2D.h"
#include "BangBang.h"
#include <QPainter>

class BangBang2D
{

public:

    Vector<Keyframe2D> ctrl;
    BangBang X;
    BangBang Y;

public:
    BangBang2D();
    ~BangBang2D(){}

    void setA(double A);
    void setV(double V);

    void clear();
    void reset();

    void addKeyframe(double dt, double x=0, double vx=0, double y=0, double vy=0);
    void addKeyframe(const Keyframe2D &kf);
    void addKeyframes(const Vector<Keyframe2D> &kfs);

    Vector<Keyframe2D> getControlSequence(bool debug=false);
    Keyframe2D evaluateAt(double dt, bool debug=false) const;
    Keyframe2D evaluateAtArcLength(double dl) const;
    double getTotalTime() const;
    double getTotalArcLength();
    void draw(QPainter* painter, const QPen& pen) const;

private:
    double arcLength(const Keyframe2D& kf, bool debug=false) const;
};

QDebug operator<<(QDebug dbg, const BangBang2D& o);
QDebug operator<<(QDebug dbg, const BangBang2D* o);

#endif // BangBang2D_H_
