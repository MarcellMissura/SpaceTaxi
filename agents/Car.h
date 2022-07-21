#ifndef CAR_H_
#define CAR_H_
#include "HolonomicObstacle.h"
#include "util/StopWatch.h"

class Car : public HolonomicObstacle
{
    int direction;
    double spawnTime;
    StopWatch spawnTimer;

public:

    Car();
    ~Car();

    virtual const QString getName() const;

    void init();
    void step();
    void spawn();
    void die();
};

#endif
