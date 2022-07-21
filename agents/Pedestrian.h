#ifndef PEDESTRIAN_H_
#define PEDESTRIAN_H_
#include "HolonomicObstacle.h"
#include "util/StopWatch.h"

class Pedestrian : public HolonomicObstacle
{
    bool dieFlag;
    double spawnTime;
    StopWatch spawnTimer;

public:

    bool shaoLin;

    Pedestrian();
    ~Pedestrian(){}

public:

    virtual const QString getName() const;

    void init();
    void reset();
    void step();

    void spawn();
    void die();

    void collisionResponse(const Obstacle* o);
};

QDataStream& operator<<(QDataStream& out, const Pedestrian &o);
QDataStream& operator>>(QDataStream& in, Pedestrian &o);

#endif
