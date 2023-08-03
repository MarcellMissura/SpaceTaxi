#ifndef HOLONOMICOBSTACLE_H_
#define HOLONOMICOBSTACLE_H_
#include "Obstacle.h"

class HolonomicObstacle : public Obstacle
{

public:

    double vx,vy,ax,ay;

public:

    HolonomicObstacle();
    ~HolonomicObstacle(){}

    HolonomicObstacle(const Hpm2D& o);
    HolonomicObstacle& operator=(const Hpm2D& v);
    operator Hpm2D() const;


    const QString getName() const;

    void physicsTransformIn();
    void physicsControl();
    void physicsTransformOut();

    Vec2 vel() const;
    void setVel(const Vec2& v);
    void setVel(double vx, double vy);
    Vec2 acc() const;
    void setAcc(const Vec2& c);
    void setAcc(double ax, double ay);

    void predict(double dt);
    HolonomicObstacle predicted(double dt) const;

    double intersects(const Hpm2D &kf) const;
    double intersects(const Unicycle &u) const;
    bool dynamicPointIntersection(const Vec2 &v, double dt) const;
    bool dynamicPolygonIntersection(const Polygon &p, double dt) const;

    Box sweptVolume(double dt) const;
};

QDebug operator<<(QDebug dbg, const HolonomicObstacle &o);
QDataStream& operator<<(QDataStream& out, const HolonomicObstacle &o);
QDataStream& operator>>(QDataStream& in, HolonomicObstacle &o);

#endif
