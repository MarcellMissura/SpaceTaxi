#include "Passenger.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/Statistics.h"
#include "util/ColorUtil.h"

// The Passenger is the little guy that appears in a drop zone.

Passenger::Passenger() : Obstacle()
{
    id = 2;
    taxiId = -1;
    dieFlag = false;
    pickupFlag = false;
    sourceDropOffPointId = -1;
    targetDropOffPointId = -1;
}

void Passenger::init()
{
    double w = config.passengerWidth;
    double h = config.passengerHeight;

    addVertex(Vec2(-w, h));
    addVertex(Vec2(-w, -h));
    addVertex(Vec2(w, -h));
    addVertex(Vec2(w, h));
}

void Passenger::reset()
{
    dieFlag = false;
    pickupFlag = false;
    sayHello = false;
    sayThankYou = false;
    tellTarget = false;
}

// Generates a human readable name.
const QString Passenger::getName() const
{
    QString name = "Passenger" + QString::number(id);
    return name;
}

void Passenger::step()
{
    //qDebug() << "Passenger step. active:" << isActive() << "taxiId:" << taxiId << "dieFlag:" << dieFlag << "pickupFlag:" << pickupFlag << "spawnTimer:" << spawnTimer.stateTime() << "dropTimer:" << dropTimer.stateTime();

    // Deactivate the speech bubble after a timeout.
    if (speechTimer.elapsedTime() > config.passengerSpeechBubbleTime)
    {
        sayHello = false;
        sayThankYou = false;
        tellTarget = false;
    }

    // Spawn again if the passenger is dead.
    if (!isActive() && taxiId < 0)
    {
        spawn();
    }

    // Drop when taxi reached target.
    else if (!isActive() && taxiId >= 0 && isAtTarget())
    {
        drop();
        state.score++;
    }

    // Die a while after drop off.
    else if (isActive() && taxiId >= 0 && dropTimer.stateTime() > config.passengerDropOffTime)
    {
        die();
    }

    // Handle die and pickup events. These are originated in collision events,
    // so they need to be handeled asynchronously, because collision event handlers
    // are called in the middle of the physics step and it's not good to mess with
    // the body at that time.
    if (dieFlag)
    {
        die();
        dieFlag = false;
    }

    if (pickupFlag)
    {
        pickup();
        pickupFlag = false;
        state.score++;
    }
}

void Passenger::spawn()
{
//    qDebug() << "Passenger spawn.";

    if (state.world.dropOffPoints.size() < 2)
        return;


    // We need to create a temporary world model for collision checking the spawn location.
    GeometricModel gm;
    gm.setFromWorld(state.world);
    gm.addStaticObstacle(*state.world.taxi);
    gm.grow(config.gmPolygonExpansionMargin);
    gm.transform();

    int newSourceDropOffId = statistics.uniformSample(0, state.world.dropOffPoints.size()-EPSILON);
    bool ok = false;
    int counter = 0;
    while (!ok && counter < 100)
    {
        ok = true;
        newSourceDropOffId = statistics.uniformSample(0, state.world.dropOffPoints.size()-EPSILON);
        if (newSourceDropOffId == targetDropOffPointId)
        {
            ok = false;
        }
        else
        {
            if (gm.pointCollisionCheck(state.world.dropOffPoints[newSourceDropOffId]) > 0)
            {
                ok = false;
                //qDebug() << "collision not okay";
            }
        }
        counter++;
    }

    if (!ok)
        return;

    int newTargetDropOffId = statistics.uniformSample(0, state.world.dropOffPoints.size()-EPSILON);
    while (newTargetDropOffId == newSourceDropOffId)
        newTargetDropOffId = statistics.uniformSample(0, state.world.dropOffPoints.size()-EPSILON);

    sourceDropOffPointId = newSourceDropOffId;
    targetDropOffPointId = newTargetDropOffId;

    taxiId = -1;
    sayHello = true;
    sayThankYou = false;
    tellTarget = false;
    speechTimer.start();

    setPos(state.world.dropOffPoints[sourceDropOffPointId]);
    physicsTransformOut();
    activate();
}

void Passenger::pickup()
{
//    qDebug() << "Passenger pickup.";

    taxiId = state.world.taxi->getId();
    sayHello = false;
    sayThankYou = false;
    tellTarget = true;
    speechTimer.start();

    deactivate();
}

void Passenger::drop()
{
    //qDebug() << "Passenger drop.";

    sayHello = false;
    sayThankYou = true;
    tellTarget = false;
    speechTimer.start();

    dropTimer.start();

    x = state.world.taxi->x+config.taxiWidth+config.passengerWidth*2;
    y = state.world.taxi->y+config.taxiHeight+config.passengerHeight*2;
    theta = 0;
    physicsTransformOut();

    activate();
}

void Passenger::die()
{
//    qDebug() << "Passenger die.";

    taxiId = -1;

    sayHello = false;
    sayThankYou = false;
    tellTarget = false;

    spawnTimer.start();

    deactivate();
}

// Returns true if the passenger is near its target drop off point.
// Closeness to target is expressed in terms of Euklidean distance.
bool Passenger::isAtTarget() const
{
    double h = Vec2(state.world.taxi->pos()-state.world.dropOffPoints[targetDropOffPointId]).norm();
    //qDebug() << "isAtTarget:" << h << config.passengerTargetRadius;
    return (h < config.passengerTargetRadius);
}

// Is called when the taxi collides with the passenger.
void Passenger::collisionResponse(const Obstacle *o)
{
    //qDebug() << "passenger collision with" << o->getName() << o->getId();

    if (isActive() && taxiId < 0 && o->getId() < 2)
    {
        pickupFlag = true;
    }

//    else if (isActive())
//    {
//        dieFlag = true;
//    }
}

void Passenger::streamOut(QDataStream& out) const
{
    out << *((Obstacle*)(this));
    out << sayHello;
    out << sayThankYou;
    out << tellTarget;
    out << sourceDropOffPointId;
    out << targetDropOffPointId;
    out << taxiId;
}

void Passenger::streamIn(QDataStream& in)
{
    in >> *((Obstacle*)(this));
    in >> sayHello;
    in >> sayThankYou;
    in >> tellTarget;
    in >> sourceDropOffPointId;
    in >> targetDropOffPointId;
    in >> taxiId;
}

QDataStream& operator<<(QDataStream& out, const Passenger &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Passenger &o)
{
    o.streamIn(in);
    return in;
}
