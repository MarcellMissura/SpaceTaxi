#ifndef PASSENGER_H_
#define PASSENGER_H_
#include "Obstacle.h"
#include "util/StopWatch.h"

class Taxi;

class Passenger : public Obstacle
{
public:

    bool sayHello;
    bool sayThankYou;
    bool tellTarget;
    bool dieFlag;
    bool pickupFlag;
    int sourceDropOffPointId;
    int targetDropOffPointId;
    int taxiId;
    StopWatch spawnTimer;
    StopWatch dropTimer;
    StopWatch speechTimer;

public:

    Passenger();
    ~Passenger(){}

public:

    virtual const QString getName() const;

    void init();
    void reset();
    void step();

    void spawn();
    void pickup();
    void drop();
    void die();
    bool isAtTarget() const;

    void collisionResponse(const Obstacle* o);

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);
};

QDataStream& operator<<(QDataStream& out, const Passenger &o);
QDataStream& operator>>(QDataStream& in, Passenger &o);

#endif
