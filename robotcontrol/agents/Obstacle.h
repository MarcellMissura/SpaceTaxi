#ifndef OBSTACLE_H_
#define OBSTACLE_H_
#include "lib/Box2D/Box2D.h"
#include "lib/geometry/Polygon.h"
#include <QPainter>

class Obstacle : public Polygon
{
protected:
    b2Body* body;
    bool active;

public:

    Obstacle();
    Obstacle(double x, double y, double w, double h);
    ~Obstacle(){}

public:

    virtual const QString getName() const;
    void setBody(b2Body* b) {body = b;}
    b2Body* getBody() const {return body;}

    virtual void activate();
    virtual void deactivate();
    bool isActive() const;

    virtual void physicsTransformIn();
    virtual void physicsControl();
    virtual void physicsTransformOut();

    virtual void init();
    virtual void reset();
    virtual void collisionResponse(const Obstacle* o);

    virtual void predict(double dt);
    Obstacle predicted(double dt) const;
};

QDebug operator<<(QDebug dbg, const Obstacle& o);
QDebug operator<<(QDebug dbg, const Obstacle* o);

#endif
