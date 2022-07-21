#include "Collision.h"

// The collision object contains the idx of an intersected obstacle,
// the idx of the ctrl bang in which the collision occured, and
// the time dt relative to the time of the beginning of the bang.
// A time of -1 indicates a no collision.

Collision::Collision()
{
    dt = -1;
    obstacleId = -1;
    obstacleIdx = -1;
    ctrlIdx = -1;
}

void Collision::clear()
{
    dt = -1;
    obstacleId = -1;
    obstacleIdx = -1;
    ctrlIdx = -1;
}

QDebug operator<<(QDebug dbg, const Collision &w)
{
    dbg << "dt:" << w.dt;
    dbg << "ctrlIdx:" << w.ctrlIdx;
    dbg << "ObstIdx (Id):" << w.obstacleIdx << "(" << w.obstacleId << ")";
    return dbg;
}
