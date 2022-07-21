#ifndef COLLISION_H_
#define COLLISION_H_
#include <QDebug>

struct Collision
{
    double dt; // Time until collision.
    int obstacleId;
    int obstacleIdx;
    int ctrlIdx;

public:

    Collision();
    ~Collision(){}

    void clear();

    bool operator<(const Collision& o) const {return (dt < o.dt);}
};

QDebug operator<<(QDebug dbg, const Collision &w);

#endif
