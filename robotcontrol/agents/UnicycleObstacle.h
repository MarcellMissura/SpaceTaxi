#ifndef UNICYCLEOBSTACLE_H_
#define UNICYCLEOBSTACLE_H_
#include "Obstacle.h"
#include "HolonomicObstacle.h"
#include "lib/pml/unicycle.h"

// The UnicycleObstacle extends the Obstacle with unicycle motion capabilities
// (acc(), vel(), predict()) and a hullPolygon that is an inflated version of
// the core Polygon the Obstacles is derived from.

class UnicycleObstacle : public Obstacle
{

public:

    Polygon hullPolygon; // Should be a bit bigger than the actual geometry.
    bool ignored;

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

    void transform();

    void setHullPolygon(const Polygon& pol);
    bool hullIntersects(const Vec2 &p, bool debug=false) const;
    bool hullIntersects(const Line &l, bool debug=false) const;
    Polygon getTransformedHull() const;
    const Polygon& getHullPolygon() const;

    void ignore();
    void unignore();
    bool isIgnored() const;

    Vec2 vel() const;
    void setVel(const Vec2& v);
    void setVel(double v, double w);
    Vec2 acc() const;
    void setAcc(const Vec2& c);
    void setAcc(double a, double b);
    void setTxVel(double vv, double ww);
    void setTxVel(const Vec2 &txVel);

    void predict(double dt);
    UnicycleObstacle predicted(double dt) const;

    void drawHull(QPainter *painter, const QPen &pen, const QBrush &brush, double opacity=0.5) const;
};

QDebug operator<<(QDebug dbg, const UnicycleObstacle &o);
QDebug operator<<(QDebug dbg, const UnicycleObstacle *o);

#endif
