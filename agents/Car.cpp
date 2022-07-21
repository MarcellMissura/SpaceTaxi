#include "Car.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/Statistics.h"
#include "util/ColorUtil.h"

// The Car is the black, boxy car that drives on the road in the game.
// It's only a kinematic body in simulation, so its physics is very
// simple: setting a constant velocity in an axis aligned direction.
// So because it's not really a car, it's derived from HolonomicObstacle
// instead of from CarObstacle.

Car::Car() : HolonomicObstacle()
{
    spawnTime = Statistics::uniformSample(0.0, config.carSpawnTime);
}

Car::~Car()
{
    //qDebug() << "Car destructor called.";
}

void Car::init()
{
    double w = config.unicycleWidth;
    double h = config.unicycleHeight;

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
}

// Generates a human readable name.
const QString Car::getName() const
{
    QString name = "Car" + QString::number(id);
    return name;
}

// The agent step function.
void Car::step()
{
    if (!active && spawnTimer.stateTime() > spawnTime)
        spawn();

    if (active && (x < 0 || x > state.world.width))
        die();
}

void Car::spawn()
{
    direction = statistics.uniformSample() > 0.5 ? 1: -1;

    x = state.world.width/2 - direction*((state.world.width/2) - 2*config.unicycleWidth);
    y = state.world.height/2 - direction*config.worldRoadHeight/2;
    theta = 0;
    vx = direction*config.carVelocity;

    physicsTransformOut();

    activate();
}

void Car::die()
{
    direction = 0;
    spawnTime = Statistics::uniformSample(0.0, config.carSpawnTime);
    spawnTimer.start();

    deactivate();
}
