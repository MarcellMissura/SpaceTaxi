#ifndef UNICYCLEOBSTACLE_H_
#define UNICYCLEOBSTACLE_H_
#include "Obstacle.h"
#include "HolonomicObstacle.h"
#include "pml/unicycle.h"

class UnicycleObstacle : public Obstacle
{

public:

    double v, w, a, b;
    double timeStep;

    UnicycleObstacle();
    ~UnicycleObstacle(){}

    operator HolonomicObstacle() const;

    UnicycleObstacle(const Unicycle& o);
    UnicycleObstacle& operator=(const Unicycle& v);
    operator Unicycle() const;

    void init();

    virtual const QString getName() const;

    void physicsTransformIn();
    void physicsControl();
    void physicsTransformOut();

    Vec2 vel() const;
    void setVel(const Vec2& v);
    void setVel(double v, double w);
    Vec2 acc() const;
    void setAcc(const Vec2& c);
    void setAcc(double a, double b);

    double intersects(const Hpm2D &kf) const;
    double intersects(const Unicycle &u) const;

    bool dynamicPointIntersection(const Vec2 &v, double dt) const;
    bool dynamicPolygonIntersection(const Polygon &p, double dt) const;

    void predict(double dt);
    UnicycleObstacle predicted(double dt) const;

    virtual const Box& boundingBox() const;
    bool isStatic() const {return false;}
};

QDebug operator<<(QDebug dbg, const UnicycleObstacle &o);

#endif
