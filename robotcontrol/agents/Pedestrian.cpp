#include "Pedestrian.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/Statistics.h"

// Pedestrians are the people (little squares)
// that walk around on the shaolin and the indoor maps.
// Pedestrians just walk straight in one direction with
// a constant velocity. In the physical simulation, the
// pedestrian is a kinematic body with a constant velocity.
// It will collide with things, but it is unaffected by
// the collision.

Pedestrian::Pedestrian() : HolonomicObstacle()
{
    id = 10000;
    dieFlag = false;
    shaoLin = false;
    spawnTime = 0;
}

void Pedestrian::init()
{
    double w = config.holonomicWidth;
    double h = config.holonomicHeight;

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
}

void Pedestrian::reset()
{
    dieFlag = false;
}

// Generates a human readable name.
const QString Pedestrian::getName() const
{
    QString name = "Pedestrian" + QString::number(id);
    return name;
}

void Pedestrian::step()
{
    // Spawn if has been dead for a while.
    if (!isActive() && spawnTimer.stateTime() > spawnTime)
        spawn();

    if (dieFlag || (x < 0 || x > state.world.width) || (y < 0 || y > state.world.height) )
    {
        die();
        dieFlag = false;
    }
}

void Pedestrian::spawn()
{
    double rx = 0;
    double ry = 0;
    bool check = false;
    while (!check)
    {
        rx = statistics.uniformSample(0, state.world.width);
        ry = statistics.uniformSample(0, state.world.height);
        Pedestrian p = *this;
        p.setPos(rx, ry);
        int obstIdx = state.world.polygonCollisionCheck(p);
        if (obstIdx < 0)
            check = true;
    }

    Vec2 rvel;
    if (shaoLin)
    {
        rvel = state.world.dropOffPoints[0]-Vec2(rx,ry);
        double v = statistics.uniformSample(0, config.pedestrianVelocity);
        rvel.normalize(v);
    }
    else
    {
        int dir = statistics.uniformSample(0.0, 4.0-EPSILON);
        if (dir == 0)
            rvel = Vec2(1, 0);
        else if (dir == 1)
            rvel = Vec2(-1, 0);
        else if (dir == 2)
            rvel = Vec2(0,1);
        if (dir == 3)
            rvel = Vec2(0,-1);

        double v = statistics.uniformSample(0, config.pedestrianVelocity);
        rvel.normalize(fabs(v));
    }

    x = rx;
    y = ry;
    theta = 0;
    vx = rvel.x;
    vy = rvel.y;
    physicsTransformOut();

    activate();
}

void Pedestrian::die()
{
    spawnTime = statistics.uniformSample(0.0, config.pedestrianSpawnTime);
    spawnTimer.start();
    deactivate();
}

// Is called when the pedestrian collides with something.
// The pedestrian dies when it collides.
void Pedestrian::collisionResponse(const Obstacle *o)
{
    //qDebug() << "Pedestrian collision" << id;

    // The pedestrian dies when it collides with anything.
    // The collision response is called in the middle of the physics step,
    // so it is not good to change the state of the body immediately. We
    // only set a flag that the object is about to die and process it in
    // the step() function with the next control loop step.
    dieFlag = true;
}

QDataStream& operator<<(QDataStream& out, const Pedestrian &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Pedestrian &o)
{
    o.streamIn(in);
    return in;
}
